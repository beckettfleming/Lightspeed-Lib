/**
 * \file lightspeed/motion/move_to_pose.hpp
 *
 * Move-to-pose (boomerang-style): drives to a full target pose (x, y,
 * heading) in one continuous curved motion, unlike DriveToPoint's discrete
 * turn-then-drive (which has no heading target at all). Steers toward a
 * "carrot" point offset behind the target along the TARGET's own heading,
 * scaled by remaining distance -- because that offset direction is fixed to
 * the target heading rather than the robot's, chasing the carrot lines the
 * robot up with that heading on arrival with no separate blend term.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
 */

#pragma once

#include <cstdint>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/motion/motion_profile.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct MoveToPoseConfig {
    // Fraction (0-1) of remaining distance the carrot sits behind the
    // target. Higher = earlier/more aggressive turn-in; lower = straighter
    // approach with a sharper final turn.
    double leadFraction;

    double positionToleranceInches;
    double headingToleranceDegrees;
    std::uint32_t settleCycles;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
    MotionProfileConfig speedProfile;  // inches/s, inches/s^2, inches/s^3, profiled over the initial straight-line distance
};

class MoveToPose {
public:
    MoveToPose(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry, const MoveToPoseConfig& config);

    // Blocking until settled within BOTH position and heading tolerance, or
    // the timeout elapses. Stops (0,0 target) on return either way.
    //
    // Known limitation (documented, not a bug): forward-only. It always
    // drives toward the carrot rather than reversing, even if the target is
    // behind the robot.
    void run(double targetX, double targetY, double targetHeadingDegrees);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    MoveToPoseConfig config_;
};

}  // namespace lightspeed::motion
