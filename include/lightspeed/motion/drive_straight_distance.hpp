/**
 * \file lightspeed/motion/drive_straight_distance.hpp
 *
 * Straight-line distance primitive: a Step 1 motion profile provides the
 * target position/velocity/acceleration envelope, a Step 2 PIDF controller
 * tracks odometry forward-distance feedback against it, and a small P-only
 * trim holds the starting heading so the move doesn't drift off-line.
 * Output feeds the same Step 2 drivetrain velocity controllers.
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
    // distancePidf tracks profiled position (inches) against measured
    // forward distance; kV should normally be ~1.0 (pass the profile's own
    // velocity straight through as feedforward) with kP/kI/kD providing
    // trim correction. settleTolerance/settleCycles define the settle
    // condition once the profile has finished.
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

    // Blocking: drives distanceInches forward (negative = backward) along
    // the heading held at the start of the call, until the profile
    // finishes and the distance PIDF settles, or the timeout elapses.
    // Commands the drivetrain throughout; stops (0,0 target) on return.
    void run(double distanceInches);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    DriveStraightDistanceConfig config_;
    control::PIDFController distancePidf_;
};

}  // namespace lightspeed::motion
