#include "lightspeed/hal/rotation_sensor.hpp"

#include "pros/error.h"

namespace lightspeed::hal {

RotationSensor::RotationSensor(std::int8_t port) : sensor_(port) {}

bool RotationSensor::isHealthy() const {
    return sensor_.get_position() != PROS_ERR;
}

double RotationSensor::getPositionCentidegrees() const {
    const std::int32_t position = sensor_.get_position();
    return position == PROS_ERR ? 0.0 : static_cast<double>(position);
}

}  // namespace lightspeed::hal
