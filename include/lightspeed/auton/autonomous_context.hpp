/**
 * \file lightspeed/auton/autonomous_context.hpp
 *
 * Bundles the shared drivetrain/motion objects a routine needs -- all
 * constructed once in src/main.cpp, all the exact same instances driver
 * control uses. A routine calls into these; it is never a separate
 * implementation of "move the robot".
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/autonomous/
 */

#pragma once

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/motion/drive_straight_distance.hpp"
#include "lightspeed/motion/drive_to_point.hpp"
#include "lightspeed/motion/move_to_pose.hpp"
#include "lightspeed/motion/pure_pursuit_controller.hpp"
#include "lightspeed/motion/turn_to_heading.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"

namespace lightspeed::auton {

struct AutonomousContext {
    control::DrivetrainVelocityController& drivetrain;
    odom::OdometryFusion& odometry;
    motion::TurnToHeading& turnToHeading;
    motion::DriveStraightDistance& driveStraightDistance;
    motion::DriveToPoint& driveToPoint;
    motion::PurePursuitController& purePursuit;
    motion::MoveToPose& moveToPose;
};

using RoutineFunction = void (*)(AutonomousContext&);

}  // namespace lightspeed::auton
