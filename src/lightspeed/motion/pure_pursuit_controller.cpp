#include "lightspeed/motion/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "lightspeed/motion/pose_math.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::motion {

namespace {

double distance(const Waypoint& a, const odom::Pose& pose) {
    return std::hypot(a.x - pose.xInches, a.y - pose.yInches);
}

// Forward-only search: never regresses to an earlier index, so the robot
// can't get stuck oscillating around a point it has already passed.
std::size_t findClosestIndexFrom(const std::vector<Waypoint>& path, const odom::Pose& pose, std::size_t startIndex) {
    std::size_t closestIndex = startIndex;
    double closestDistance = distance(path[startIndex], pose);
    for (std::size_t i = startIndex + 1; i < path.size(); ++i) {
        const double d = distance(path[i], pose);
        if (d < closestDistance) {
            closestDistance = d;
            closestIndex = i;
        }
    }
    return closestIndex;
}

// First path point at or beyond lookaheadDistance from the robot, searching
// forward from startIndex; the path's last point if none qualifies (i.e.
// the robot is within lookahead of the goal -- aim straight at it).
Waypoint findLookaheadPoint(const std::vector<Waypoint>& path, const odom::Pose& pose, std::size_t startIndex,
                             double lookaheadDistance) {
    for (std::size_t i = startIndex; i < path.size(); ++i) {
        if (distance(path[i], pose) >= lookaheadDistance) {
            return path[i];
        }
    }
    return path.back();
}

}  // namespace

PurePursuitController::PurePursuitController(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                                               const PurePursuitConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config) {}

void PurePursuitController::follow(const std::vector<Waypoint>& rawWaypoints) {
    if (rawWaypoints.size() < 2) {
        return;
    }

    const std::vector<Waypoint> path = buildSmoothPath(rawWaypoints, config_.smoothing);

    double totalPathLength = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        totalPathLength += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    const MotionProfile speedProfile(0.0, totalPathLength, config_.speedProfile);

    std::size_t searchIndex = 0;
    std::uint32_t settledCycles = 0;
    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const odom::Pose pose = odometry_.getPose();
        const odom::Velocity velocity = odometry_.getVelocity();
        const double currentSpeed = std::hypot(velocity.xInchesPerSecond, velocity.yInchesPerSecond);

        const double lookaheadDistance = std::clamp(config_.minLookaheadInches + config_.lookaheadSpeedGain * currentSpeed,
                                                      config_.minLookaheadInches, config_.maxLookaheadInches);

        searchIndex = findClosestIndexFrom(path, pose, searchIndex);
        const Waypoint lookaheadPoint = findLookaheadPoint(path, pose, searchIndex, lookaheadDistance);

        const LocalOffset local = toLocalFrame(pose, lookaheadPoint.x, lookaheadPoint.y);
        const double lSquared = local.forward * local.forward + local.strafeRight * local.strafeRight;
        const double curvature = lSquared > 1e-6 ? (2.0 * local.strafeRight / lSquared) : 0.0;

        const double targetSpeed = speedProfile.sample(elapsedSeconds).velocity;

        const WheelSpeeds wheelSpeeds = curvatureToWheelSpeeds(targetSpeed, curvature, config_.kinematics.trackWidthInches);
        drivetrain_.setTargetVelocity(inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics),
                                       inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics));

        const double distanceToEnd = distanceToPoint(pose, path.back().x, path.back().y);
        settledCycles = distanceToEnd <= config_.positionToleranceInches ? settledCycles + 1 : 0;

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf("[purePursuit] pose=(%.1f, %.1f, %.1fdeg) lookahead=(%.1f, %.1f) curvature=%.4f speed=%.1f\n",
                        pose.xInches, pose.yInches, pose.headingDegrees, lookaheadPoint.x, lookaheadPoint.y, curvature,
                        targetSpeed);
            previousPrintTime = now;
        }

        if (settledCycles >= config_.settleCycles || elapsedSeconds >= config_.timeoutSeconds) {
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
}

}  // namespace lightspeed::motion
