/**
 * \file lightspeed/hal/imu.hpp
 *
 * Generic HAL wrapper around a single V5 Inertial Sensor. Dual-IMU
 * averaging lives one layer up, in lightspeed::odom::IMUSource.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/hal/
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

    // Heading reads are meaningless before this is true -- a match must
    // never start trusting a mid-calibration heading.
    [[nodiscard]] bool isReady() const;

    // [0, 360), clockwise-positive. 0.0 if the sensor isn't ready.
    // Display only -- use getContinuousHeadingDegrees() for delta math.
    [[nodiscard]] double getHeadingDegrees() const;

    // Unbounded (no [0,360) wraparound), clockwise-positive. 0.0 if the
    // sensor isn't ready. Use this when computing a delta between reads.
    [[nodiscard]] double getContinuousHeadingDegrees() const;

private:
    pros::Imu sensor_;
};

}  // namespace lightspeed::hal
