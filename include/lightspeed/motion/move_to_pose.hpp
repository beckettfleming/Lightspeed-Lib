/**
 * \file lightspeed/motion/move_to_pose.hpp
 *
 * Move-to-pose (boomerang-style): drives to a full target pose (x, y,
 * heading) in one continuous curved motion, rather than DriveToPoint's
 * discrete turn-then-drive (which also has no heading target at all --
 * Waypoint/PurePursuitController don't either). Steers via curvature toward
 * a "carrot" point offset behind the target along the TARGET's own heading,
 * scaled by remaining distance -- the classic boomerang technique: the
 * carrot converges on the target as the robot closes in, and because the
 * offset direction is fixed to the target heading throughout (not the
 * robot's own), chasing it naturally lines the robot up with that heading
 * on arrival, without a separate heading-blend term. Reuses the same
 * curvature-steering math as PurePursuitController and feeds the same Step
 * 2 drivetrain velocity controllers everything else does.
 */

#pragma once

#include <cstdint>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/motion/motion_profile.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct MoveToPoseConfig {
    // Carrot-point lead, as a fraction (0-1) of the remaining straight-line
    // distance to the target, offset behind it along the target heading.
    // Higher = earlier/more aggressive turn-in, lower = straighter approach
    // with a sharper final turn to face targetHeadingDegrees.
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

    // Blocking: drives to (targetX, targetY, targetHeadingDegrees) via a
    // single continuous curved motion, until the robot settles within both
    // position AND heading tolerance, or the timeout elapses. Commands the
    // drivetrain throughout; stops (0,0 target) on return either way.
    //
    // Known limitation (documented, not a bug): like PurePursuitController,
    // this is forward-only -- it always drives toward the carrot point
    // rather than reversing, even if the target pose is behind the robot's
    // current heading. That matches this project's other curvature-steered
    // primitives; a target that genuinely requires backing up isn't handled
    // specially.
    void run(double targetX, double targetY, double targetHeadingDegrees);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    MoveToPoseConfig config_;
};

}  // namespace lightspeed::motion
