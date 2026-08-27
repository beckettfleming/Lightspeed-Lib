#include "lightspeed/odom/ime_source.hpp"

namespace lightspeed::odom {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

IMESource::IMESource(hal::MotorGroup& motors, const IMEConfig& config) : motors_(motors), config_(config) {}

bool IMESource::isHealthy() const {
    return motors_.getHealth() != hal::HealthStatus::disconnected;
}

void IMESource::resetBaseline() {
    previousPositionDeg_ = motors_.getPositionRaw();
    hasBaseline_ = true;
}

double IMESource::readDeltaInches() {
    if (!isHealthy()) {
        return 0.0;
    }

    const double currentPositionDeg = motors_.getPositionRaw();
    if (!hasBaseline_) {
        previousPositionDeg_ = currentPositionDeg;
        hasBaseline_ = true;
        return 0.0;
    }

    const double deltaMotorDeg = currentPositionDeg - previousPositionDeg_;
    previousPositionDeg_ = currentPositionDeg;

    const double wheelRevolutions = (deltaMotorDeg / 360.0) * config_.gearRatio;
    return wheelRevolutions * (kPi * config_.wheelDiameterInches);
}

}  // namespace lightspeed::odom
