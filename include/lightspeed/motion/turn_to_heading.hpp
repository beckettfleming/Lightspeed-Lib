/**
 * \file lightspeed/motion/turn_to_heading.hpp
 *
 * Rotation-in-place primitive: a PIDF controller on odometry heading
 * feedback, whose output is an angular velocity command converted to
 * left/right wheel RPM.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
 */

#pragma once

#include <cstdint>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/control/pidf_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct TurnToHeadingConfig {
    // kV/kA/kS are normally 0 here -- no feedforward target velocity for a
    // static heading target.
    control::PIDFConfig pidf;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
};

class TurnToHeading {
public:
    TurnToHeading(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                  const TurnToHeadingConfig& config);

    // Blocking. targetHeadingDegrees is clockwise-positive. Stops (0,0
    // target) on return, settled or timed out.
    void run(double targetHeadingDegrees);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    TurnToHeadingConfig config_;
    control::PIDFController pidf_;
};

}  // namespace lightspeed::motion
