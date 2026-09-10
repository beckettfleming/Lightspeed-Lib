/**
 * \file lightspeed/hal/motor_group.hpp
 *
 * Generic HAL wrapper around a ganged group of V5 motors, controlled by raw
 * voltage. Construct one per logical device named in lightspeed::hal::config.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/hal/
 */

#pragma once

#include <cstdint>

#include "lightspeed/hal/config.hpp"
#include "pros/motor_group.hpp"

namespace lightspeed::hal {

enum class HealthStatus : std::uint8_t {
    ok = 0,
    stalled,          // commanded significant voltage but isn't turning
    overTemperature,
    disconnected,
};

const char* toString(HealthStatus status);

// All reads below are cheap, side-effect-free queries, safe to call from
// multiple tasks at different rates.
class MotorGroup {
public:
    explicit MotorGroup(const config::MotorGroupConfig& config);

    // Averaged across the group, in this group's configured encoder units.
    // Raw -- inch/gear-ratio conversion belongs to a higher layer.
    [[nodiscard]] double getPositionRaw() const;

    // Averaged across the group, in RPM.
    [[nodiscard]] double getVelocityRpm() const;

    // Clamped to the V5 motor's supported range (+-12000 mV).
    void writeVoltage(std::int32_t millivolts) const;

    // Deliberately coarse: `disconnected` is reported the instant even ONE
    // motor drops out. Callers that need to tell a partial failure from a
    // total one must use getConnectedMotorCount() alongside this, not
    // instead of it.
    [[nodiscard]] HealthStatus getHealth() const;

    // Total motors configured, fixed at construction.
    [[nodiscard]] std::uint8_t getMotorCount() const;

    // How many are reporting a finite temperature (i.e. not disconnected)
    // right now. Compare against getMotorCount() to distinguish a partial
    // failure from a total one.
    [[nodiscard]] std::uint8_t getConnectedMotorCount() const;

    [[nodiscard]] const char* name() const {
        return name_;
    }

private:
    const char* name_;
    pros::MotorGroup group_;
};

}  // namespace lightspeed::hal
