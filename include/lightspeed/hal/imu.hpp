/**
 * \file lightspeed/hal/imu.hpp
 *
 * Generic HAL wrapper around a single V5 Inertial Sensor. Dual-IMU
 * averaging (when a robot has two) lives one layer up, in
 * lightspeed::odom::IMUSource -- this wrapper only ever talks to one
 * physical sensor.
 */

#pragma once

#include <cstdint>

#include "pros/imu.hpp"

namespace lightspeed::hal {

class Imu {
public:
    explicit Imu(std::uint8_t port);

    // Starts (re-)calibration. Blocks for ~2-3s if blocking is true.
    // isReady() stays false until calibration completes.
    void calibrate(bool blocking = false) const;

    // True once startup/re-calibration has finished and the sensor is
    // reporting valid data. Heading reads are meaningless before this is
    // true -- a match must never start trusting a mid-calibration heading.
    [[nodiscard]] bool isReady() const;

    // Heading in degrees, bounded to [0, 360), clockwise-positive (VEXos's
    // native convention). Returns 0.0 if the sensor isn't ready.
    [[nodiscard]] double getHeadingDegrees() const;

    // Total accumulated rotation in degrees, clockwise-positive,
    // theoretically unbounded (no [0,360) wraparound) -- use this rather
    // than getHeadingDegrees() when computing a delta between two reads.
    // Returns 0.0 if the sensor isn't ready.
    [[nodiscard]] double getContinuousHeadingDegrees() const;

private:
    pros::Imu sensor_;
};

}  // namespace lightspeed::hal
