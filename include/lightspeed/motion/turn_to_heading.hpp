/**
 * \file lightspeed/motion/turn_to_heading.hpp
 *
 * Rotation-in-place primitive: a Step 2 PIDF controller operating on
 * odometry heading feedback, whose output is an angular velocity command
 * converted to left/right wheel RPM and fed into the existing Step 2
 * drivetrain velocity controllers -- same output path as everything else.
 */

#pragma once

#include <cstdint>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/control/pidf_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct TurnToHeadingConfig {
    // pidf.settleTolerance/settleCycles (degrees / cycle count) define the
    // settle condition; pidf.kV/kA/kS are normally 0 here (no feedforward
    // target velocity for a static heading target).
    control::PIDFConfig pidf;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
};

class TurnToHeading {
public:
    TurnToHeading(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                  const TurnToHeadingConfig& config);

    // Blocking: rotates in place to targetHeadingDegrees (clockwise-positive,
    // matching lightspeed::odom's convention) until settled or timed out.
    // Commands the drivetrain throughout; stops (0,0 target) on return.
    void run(double targetHeadingDegrees);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    TurnToHeadingConfig config_;
    control::PIDFController pidf_;
};

}  // namespace lightspeed::motion
