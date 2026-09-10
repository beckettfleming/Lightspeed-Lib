/**
 * \file lightspeed/motion/drive_straight_distance.hpp
 *
 * Straight-line distance primitive: a motion profile provides the
 * position/velocity/acceleration envelope, a PIDF controller tracks odometry
 * forward-distance against it, and a P-only trim holds the starting heading
 * so the move doesn't drift off-line.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
 */

#pragma once

#include <cstdint>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/control/pidf_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/motion/motion_profile.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct DriveStraightDistanceConfig {
    // Tracks profiled position (inches) against measured forward distance.
    // kV should normally be ~1.0 -- pass the profile's own velocity straight
    // through as feedforward, with kP/kI/kD providing trim only.
    control::PIDFConfig distancePidf;
    MotionProfileConfig motionProfile;  // inches/s, inches/s^2, inches/s^3
    double headingCorrectionKP;         // degrees of drift -> deg/s correction trim
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
};

class DriveStraightDistance {
public:
    DriveStraightDistance(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                           const DriveStraightDistanceConfig& config);

    // Blocking. Negative distance = backward. Holds the heading captured at
    // the start of the call. Stops (0,0 target) on return.
    void run(double distanceInches);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    DriveStraightDistanceConfig config_;
    control::PIDFController distancePidf_;
};

}  // namespace lightspeed::motion
