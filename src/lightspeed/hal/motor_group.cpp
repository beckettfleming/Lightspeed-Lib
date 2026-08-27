#include "lightspeed/hal/motor_group.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace lightspeed::hal {

namespace {

constexpr std::int32_t kMaxVoltageMillivolts = 12000;

// A motor is considered stalled when it's being commanded a meaningful
// fraction of full voltage but isn't actually turning -- e.g. jammed
// against a hard stop or badly overloaded.
constexpr double kStallVoltageFraction = 0.25;  // 25% of 12000 mV = 3000 mV
constexpr double kStallVelocityRpmThreshold = 3.0;

// Averages only finite readings so one disconnected/erroring motor in the
// group (which reports PROS_ERR_F, i.e. infinity) doesn't poison the whole
// group's reading.
double averageFinite(const std::vector<double>& values) {
    double sum = 0.0;
    int count = 0;
    for (double value : values) {
        if (std::isfinite(value)) {
            sum += value;
            ++count;
        }
    }
    return count > 0 ? sum / count : 0.0;
}

}  // namespace

MotorGroup::MotorGroup(const config::MotorGroupConfig& config)
    : name_(config.name), group_(config.ports, config.gearset, config.encoderUnits) {}

double MotorGroup::getPositionRaw() const {
    return averageFinite(group_.get_position_all());
}

double MotorGroup::getVelocityRpm() const {
    return averageFinite(group_.get_actual_velocity_all());
}

void MotorGroup::writeVoltage(std::int32_t millivolts) const {
    const std::int32_t clamped = std::clamp(millivolts, -kMaxVoltageMillivolts, kMaxVoltageMillivolts);
    group_.move_voltage(clamped);
}

HealthStatus MotorGroup::getHealth() const {
    // Checked first: a disconnected motor reports PROS_ERR_F (infinity) for
    // temperature, which would otherwise also look "not over temp" and
    // "not stalled".
    for (double temperatureC : group_.get_temperature_all()) {
        if (!std::isfinite(temperatureC)) {
            return HealthStatus::disconnected;
        }
    }

    for (std::int32_t overTemp : group_.is_over_temp_all()) {
        if (overTemp > 0) {
            return HealthStatus::overTemperature;
        }
    }

    const auto voltages = group_.get_voltage_all();
    const auto velocities = group_.get_actual_velocity_all();
    const std::size_t motorCount = std::min(voltages.size(), velocities.size());
    for (std::size_t i = 0; i < motorCount; ++i) {
        const double commandedFraction = std::abs(voltages[i]) / static_cast<double>(kMaxVoltageMillivolts);
        if (commandedFraction >= kStallVoltageFraction && std::abs(velocities[i]) < kStallVelocityRpmThreshold) {
            return HealthStatus::stalled;
        }
    }

    return HealthStatus::ok;
}

std::uint8_t MotorGroup::getMotorCount() const {
    return static_cast<std::uint8_t>(group_.get_temperature_all().size());
}

std::uint8_t MotorGroup::getConnectedMotorCount() const {
    std::uint8_t count = 0;
    for (double temperatureC : group_.get_temperature_all()) {
        if (std::isfinite(temperatureC)) {
            ++count;
        }
    }
    return count;
}

const char* toString(HealthStatus status) {
    switch (status) {
        case HealthStatus::ok:
            return "OK";
        case HealthStatus::stalled:
            return "STALLED";
        case HealthStatus::overTemperature:
            return "OVER_TEMP";
        case HealthStatus::disconnected:
            return "DISCONNECTED";
    }
    return "UNKNOWN";
}

}  // namespace lightspeed::hal
