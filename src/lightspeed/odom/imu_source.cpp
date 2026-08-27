#include "lightspeed/odom/imu_source.hpp"

namespace lightspeed::odom {

IMUSource::IMUSource(hal::Imu& primary, hal::Imu* secondary) : primary_(primary), secondary_(secondary) {}

bool IMUSource::isHealthy() const {
    return primary_.isReady() || (secondary_ != nullptr && secondary_->isReady());
}

double IMUSource::currentContinuousHeading() const {
    const bool primaryReady = primary_.isReady();
    const bool secondaryReady = secondary_ != nullptr && secondary_->isReady();

    if (primaryReady && secondaryReady) {
        return (primary_.getContinuousHeadingDegrees() + secondary_->getContinuousHeadingDegrees()) / 2.0;
    }
    if (primaryReady) {
        return primary_.getContinuousHeadingDegrees();
    }
    if (secondaryReady) {
        return secondary_->getContinuousHeadingDegrees();
    }
    // Neither ready: hold the last known value so the delta reads as 0
    // rather than swinging wildly once a sensor comes back.
    return previousRotationDeg_;
}

void IMUSource::resetBaseline() {
    previousRotationDeg_ = currentContinuousHeading();
    hasBaseline_ = true;
}

double IMUSource::readHeadingDeltaDegrees() {
    const double current = currentContinuousHeading();
    if (!hasBaseline_) {
        previousRotationDeg_ = current;
        hasBaseline_ = true;
        return 0.0;
    }
    const double delta = current - previousRotationDeg_;
    previousRotationDeg_ = current;
    return delta;
}

double IMUSource::getHeadingDegrees() const {
    if (primary_.isReady()) {
        return primary_.getHeadingDegrees();
    }
    if (secondary_ != nullptr && secondary_->isReady()) {
        return secondary_->getHeadingDegrees();
    }
    return 0.0;
}

}  // namespace lightspeed::odom
