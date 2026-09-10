/**
 * \file lightspeed/odom/imu_source.hpp
 *
 * Heading source backed by one or two hal::Imu wrappers, averaging when both
 * are configured and ready. The fusion core's primary heading input.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/odometry/
 */

#pragma once

#include "lightspeed/hal/imu.hpp"

namespace lightspeed::odom {

class IMUSource {
public:
    // secondary may be nullptr. When both are present and ready their
    // continuous-rotation readings are averaged, which assumes both were
    // calibrated together so their references start aligned.
    explicit IMUSource(hal::Imu& primary, hal::Imu* secondary = nullptr);

    // Degrees, clockwise-positive, since the last read. Zero on the first
    // read after construction/resetBaseline().
    double readHeadingDeltaDegrees();

    // True if at least one configured IMU is ready.
    [[nodiscard]] bool isHealthy() const;

    void resetBaseline();

    // [0,360), display only -- delta math uses continuous rotation.
    [[nodiscard]] double getHeadingDegrees() const;

private:
    [[nodiscard]] double currentContinuousHeading() const;

    hal::Imu& primary_;
    hal::Imu* secondary_;
    double previousRotationDeg_ = 0.0;
    bool hasBaseline_ = false;
};

}  // namespace lightspeed::odom
