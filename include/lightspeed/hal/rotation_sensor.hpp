/**
 * \file lightspeed/hal/rotation_sensor.hpp
 *
 * Generic HAL wrapper around a single V5 Rotation Sensor, used by odometry
 * tracking-wheel pods.
 */

#pragma once

#include <cstdint>

#include "pros/rotation.hpp"

namespace lightspeed::hal {

class RotationSensor {
public:
    // port: negative reverses the sensor's direction, matching PROS's own
    // convention (see pros::Rotation).
    explicit RotationSensor(std::int8_t port);

    // Raw position in centidegrees (0.01 degree units) -- unconverted.
    // Inch/degree conversion happens in odometry sources, not here.
    // Returns 0.0 if the sensor isn't reporting valid data.
    [[nodiscard]] double getPositionCentidegrees() const;

    [[nodiscard]] bool isHealthy() const;

private:
    pros::Rotation sensor_;
};

}  // namespace lightspeed::hal
