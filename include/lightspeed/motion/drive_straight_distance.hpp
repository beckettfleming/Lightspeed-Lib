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

    // Overshoot guard. Every cycle, speed TOWARD the target is capped at
    // sqrt(2 * maxBrakingDecel * remaining), where remaining is measured from
    // odometry and shortened by (current speed * brakingLagSeconds) to cover
    // the drivetrain's response delay. Unlike the profile (which is
    // time-based and can't see that the robot got ahead of it), this uses
    // where the robot actually is, so it brakes in time even if the robot is
    // running fast. Set maxBrakingDecel to what the robot can REALLY do on
    // the ground -- lower = safer, slower final approach. 0 disables.
    double maxBrakingDecel = 0.0;      // inches/s^2
    double brakingLagSeconds = 0.0;    // drivetrain response delay
};

class DriveStraightDistance {
public:
    DriveStraightDistance(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                           const DriveStraightDistanceConfig& config);

    // Blocking. Negative distance = backward. Holds the heading captured at
    // the start of the call unless holdHeading is false. Stops (0,0 target)
    // on return.
    void run(double distanceInches, bool holdHeading = true);
    // useOvershootGuard = false skips the maxBrakingDecel cap for this move --
    // for deliberate impacts (e.g. bumping a field toggle) where arriving
    // with speed is the point. timeoutSeconds > 0 overrides the configured
    // timeout -- for moves that may end pressed against something and can't
    // physically reach their distance.
    void run(double distanceInches, bool holdHeading, const MotionProfileConfig& motionProfile,
             bool useOvershootGuard = true, double timeoutSeconds = 0.0);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    DriveStraightDistanceConfig config_;
    control::PIDFController distancePidf_;
};

}  // namespace lightspeed::motion
