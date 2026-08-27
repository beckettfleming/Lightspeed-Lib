#include "lightspeed/motion/pose_math.hpp"

#include <cmath>

namespace lightspeed::motion {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
}  // namespace

LocalOffset toLocalFrame(const odom::Pose& robotPose, double targetX, double targetY) {
    const double dx = targetX - robotPose.xInches;
    const double dy = targetY - robotPose.yInches;
    const double headingRadians = robotPose.headingDegrees * kDegreesToRadians;

    // Inverse of OdometryFusion's local->field rotation
    // (dx = forward*sin(h) + strafe*cos(h), dy = forward*cos(h) - strafe*sin(h)).
    // That rotation matrix is its own inverse here, so this is the same
    // form applied to (dx, dy) instead of (forward, strafe).
    return LocalOffset{
        .forward = dx * std::sin(headingRadians) + dy * std::cos(headingRadians),
        .strafeRight = dx * std::cos(headingRadians) - dy * std::sin(headingRadians),
    };
}

double distanceToPoint(const odom::Pose& robotPose, double targetX, double targetY) {
    const double dx = targetX - robotPose.xInches;
    const double dy = targetY - robotPose.yInches;
    return std::hypot(dx, dy);
}

double headingToPoint(const odom::Pose& robotPose, double targetX, double targetY) {
    const double dx = targetX - robotPose.xInches;
    const double dy = targetY - robotPose.yInches;
    // Clockwise-from-+y heading, equivalent to atan2 with the axes swapped
    // relative to the standard counterclockwise-from-+x convention.
    double headingDegrees = std::atan2(dx, dy) * kRadiansToDegrees;
    if (headingDegrees < 0.0) {
        headingDegrees += 360.0;
    }
    return headingDegrees;
}

double headingErrorDegrees(double from, double to) {
    double diff = std::fmod(to - from + 180.0, 360.0);
    if (diff < 0.0) {
        diff += 360.0;
    }
    return diff - 180.0;
}

}  // namespace lightspeed::motion
