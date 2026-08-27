#include "lightspeed/hal/imu.hpp"

#include <cmath>

namespace lightspeed::hal {

Imu::Imu(std::uint8_t port) : sensor_(port) {}

void Imu::calibrate(bool blocking) const {
    sensor_.reset(blocking);
}

bool Imu::isReady() const {
    return !sensor_.is_calibrating() && std::isfinite(sensor_.get_heading());
}

double Imu::getHeadingDegrees() const {
    return isReady() ? sensor_.get_heading() : 0.0;
}

double Imu::getContinuousHeadingDegrees() const {
    return isReady() ? sensor_.get_rotation() : 0.0;
}

}  // namespace lightspeed::hal
