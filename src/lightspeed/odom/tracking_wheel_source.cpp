#include "lightspeed/odom/tracking_wheel_source.hpp"

namespace lightspeed::odom {

TrackingWheelSource::TrackingWheelSource(hal::RotationSensor& sensor, const PodConfig& config)
    : sensor_(sensor), config_(config) {}

bool TrackingWheelSource::isHealthy() const {
    return sensor_.isHealthy();
}

void TrackingWheelSource::resetBaseline() {
    previousPositionCentidegrees_ = sensor_.getPositionCentidegrees();
    hasBaseline_ = true;
}

double TrackingWheelSource::readDeltaInches() {
    if (!isHealthy()) {
        return 0.0;
    }

    const double currentPosition = sensor_.getPositionCentidegrees();
    if (!hasBaseline_) {
        previousPositionCentidegrees_ = currentPosition;
        hasBaseline_ = true;
        return 0.0;
    }

    const double deltaCentidegrees = currentPosition - previousPositionCentidegrees_;
    previousPositionCentidegrees_ = currentPosition;
    return deltaCentidegrees * config_.ticksToInches;
}

}  // namespace lightspeed::odom
