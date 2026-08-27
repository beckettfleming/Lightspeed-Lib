/**
 * \file lightspeed/odom/imu_source.hpp
 *
 * Heading source backed by one or two hal::Imu wrappers, averaging when
 * both are configured and ready. This is the odometry fusion core's
 * primary heading input.
 */

#pragma once

#include "lightspeed/hal/imu.hpp"

namespace lightspeed::odom {

class IMUSource {
public:
    // secondary may be nullptr if the robot only has one IMU. When both are
    // present and ready, their continuous-rotation readings are averaged --
    // this assumes both were calibrated/tared together so their rotation
    // references start aligned (see hal::Imu::calibrate).
    explicit IMUSource(hal::Imu& primary, hal::Imu* secondary = nullptr);

    // Heading delta (degrees, clockwise-positive) since the last read.
    // Zero on the first read after construction/resetBaseline().
    double readHeadingDeltaDegrees();

    // True if at least one configured IMU is ready.
    [[nodiscard]] bool isHealthy() const;

    void resetBaseline();

    // Current absolute heading (degrees, [0,360)) for display -- not used
    // internally for delta math (continuous rotation is used for that to
    // avoid the [0,360) wraparound).
    [[nodiscard]] double getHeadingDegrees() const;

private:
    [[nodiscard]] double currentContinuousHeading() const;

    hal::Imu& primary_;
    hal::Imu* secondary_;
    double previousRotationDeg_ = 0.0;
    bool hasBaseline_ = false;
};

}  // namespace lightspeed::odom
