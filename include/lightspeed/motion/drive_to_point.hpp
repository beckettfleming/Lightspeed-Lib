/**
 * \file lightspeed/motion/drive_to_point.hpp
 *
 * Drive-to-point / turn-to-point: computes heading/distance to a target
 * point from the current odometry pose, then applies TurnToHeading and
 * DriveStraightDistance -- no new control loop, purely composition.
 */

#pragma once

#include "lightspeed/motion/drive_straight_distance.hpp"
#include "lightspeed/motion/turn_to_heading.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::motion {

class DriveToPoint {
public:
    DriveToPoint(TurnToHeading& turnToHeading, DriveStraightDistance& driveStraightDistance, odom::OdometryFusion& odometry);

    // Rotates in place to face (targetX, targetY).
    void turnToPoint(double targetX, double targetY);

    // Turns to face (targetX, targetY), then drives straight to it.
    void driveToPoint(double targetX, double targetY);

private:
    TurnToHeading& turnToHeading_;
    DriveStraightDistance& driveStraightDistance_;
    odom::OdometryFusion& odometry_;
};

}  // namespace lightspeed::motion
