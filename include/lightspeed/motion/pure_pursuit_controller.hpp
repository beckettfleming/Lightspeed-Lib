/**
 * \file lightspeed/motion/pure_pursuit_controller.hpp
 *
 * Pure pursuit path following: adaptive lookahead + curvature steering,
 * with a Step 1 motion profile providing the target speed envelope over
 * the path's duration. Feeds the same Step 2 drivetrain velocity
 * controllers driver control uses -- one output path regardless of source.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/motion/motion_profile.hpp"
#include "lightspeed/motion/path.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

struct PurePursuitConfig {
    double minLookaheadInches;
    double maxLookaheadInches;
    double lookaheadSpeedGain;  // lookahead = clamp(min + gain*currentSpeed, min, max)
    double positionToleranceInches;
    std::uint32_t settleCycles;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
    MotionProfileConfig speedProfile;  // inches/s, inches/s^2, inches/s^3 along total path length
    PathSmoothingConfig smoothing;
};

class PurePursuitController {
public:
    PurePursuitController(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                           const PurePursuitConfig& config);

    // Blocking: smooths rawWaypoints, then follows the path until the robot
    // settles at the final point or the timeout elapses. Commands the
    // drivetrain velocity controllers throughout; stops (0,0 target) on
    // return either way. No-op if rawWaypoints has fewer than 2 points.
    void follow(const std::vector<Waypoint>& rawWaypoints);

private:
    control::DrivetrainVelocityController& drivetrain_;
    odom::OdometryFusion& odometry_;
    PurePursuitConfig config_;
};

}  // namespace lightspeed::motion
