#include "lightspeed/motion/drive_to_point.hpp"

#include "lightspeed/motion/pose_math.hpp"

namespace lightspeed::motion {

DriveToPoint::DriveToPoint(TurnToHeading& turnToHeading, DriveStraightDistance& driveStraightDistance, odom::OdometryFusion& odometry)
    : turnToHeading_(turnToHeading), driveStraightDistance_(driveStraightDistance), odometry_(odometry) {}

void DriveToPoint::turnToPoint(double targetX, double targetY) {
    const odom::Pose pose = odometry_.getPose();
    turnToHeading_.run(headingToPoint(pose, targetX, targetY));
}

void DriveToPoint::driveToPoint(double targetX, double targetY) {
    const odom::Pose pose = odometry_.getPose();
    turnToHeading_.run(headingToPoint(pose, targetX, targetY));
    driveStraightDistance_.run(distanceToPoint(pose, targetX, targetY));
}

}  // namespace lightspeed::motion
