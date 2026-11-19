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
//
// Windowed: only looks maxSearchInches of path length ahead of startIndex.
// An unbounded search lets a robot that drifts off a curved path "teleport"
// its progress to a later section that happens to be nearby (e.g. the end of
// a hook), after which it aims straight at the endpoint and skips the curve.
std::size_t findClosestIndexFrom(const std::vector<Waypoint>& path, const odom::Pose& pose, std::size_t startIndex,
                                 double maxSearchInches) {
    std::size_t closestIndex = startIndex;
    double closestDistance = distance(path[startIndex], pose);
    double searchedInches = 0.0;
    for (std::size_t i = startIndex + 1; i < path.size(); ++i) {
        searchedInches += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        if (searchedInches > maxSearchInches) {
            break;
        }
        const double d = distance(path[i], pose);
        if (d < closestDistance) {
            closestDistance = d;
            closestIndex = i;
        }
    }
    return closestIndex;
}

// First path point at or beyond lookaheadDistance from the robot, searching
// forward from startIndex. If none qualifies (the robot is within lookahead
// of the end), aim at a point on the final segment's line EXTENDED past the
// end, still ~lookaheadDistance away. Aiming at the endpoint itself makes
// curvature (2*offset/L^2) blow up as L shrinks: an inch from the end, a
// fraction of an inch of sideways offset demands a near-pivot, and the robot
// spins in place instead of finishing (seen 2026-11-19: curvature -0.20,
// stuck 1.23in from the end until timeout). Arrival is still decided by
// distance to the real endpoint, so this only changes steering.
Waypoint findLookaheadPoint(const std::vector<Waypoint>& path, const odom::Pose& pose, std::size_t startIndex,
                             double lookaheadDistance) {
    for (std::size_t i = startIndex; i < path.size(); ++i) {
        if (distance(path[i], pose) >= lookaheadDistance) {
            return path[i];
        }
    }
    const Waypoint& end = path.back();
    if (path.size() < 2) {
        return end;
    }
    const Waypoint& beforeEnd = path[path.size() - 2];
    const double segmentLength = std::hypot(end.x - beforeEnd.x, end.y - beforeEnd.y);
    if (segmentLength < 1e-6) {
        return end;
    }
    const double extension = std::max(0.0, lookaheadDistance - distance(end, pose));
    return Waypoint{end.x + (end.x - beforeEnd.x) / segmentLength * extension,
                    end.y + (end.y - beforeEnd.y) / segmentLength * extension};
}

}  // namespace

PurePursuitController::PurePursuitController(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                                               const PurePursuitConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config) {}

void PurePursuitController::follow(const std::vector<Waypoint>& rawWaypoints, bool backwards, std::uint32_t settleCycles,
                                   double lookaheadLimitInches, double positionToleranceInches, double timeoutSeconds) {
    if (rawWaypoints.size() < 2) {
        return;
    }

    const std::vector<Waypoint> path = buildSmoothPath(rawWaypoints, config_.smoothing);

    double totalPathLength = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        totalPathLength += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    const MotionProfile speedProfile(0.0, totalPathLength, config_.speedProfile);
    const std::uint32_t requiredSettleCycles = settleCycles == 0 ? config_.settleCycles : settleCycles;
    const double endToleranceInches =
        positionToleranceInches > 0.0 ? positionToleranceInches : config_.positionToleranceInches;
    const double moveTimeoutSeconds = timeoutSeconds > 0.0 ? timeoutSeconds : config_.timeoutSeconds;
    const double maxLookaheadInches =
        lookaheadLimitInches > 0.0 ? std::clamp(lookaheadLimitInches, config_.minLookaheadInches, config_.maxLookaheadInches)
                                   : config_.maxLookaheadInches;

    std::size_t searchIndex = 0;
    std::uint32_t settledCycles = 0;
    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;
    constexpr double kTerminalApproachSpeedInchesPerSecond = 24.0;
    constexpr double kTerminalDecelerationInchesPerSecondSquared = 30.0;
    constexpr double kTerminalControlDistanceInches = 12.0;
    constexpr double kMinApproachSpeedInchesPerSecond = 4.0;  // ~28 motor rpm; below ~15-20 the drive sticks

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const odom::Pose pose = odometry_.getPose();
        const odom::Velocity velocity = odometry_.getVelocity();
        const double currentSpeed = std::hypot(velocity.xInchesPerSecond, velocity.yInchesPerSecond);

        const double lookaheadDistance = std::clamp(config_.minLookaheadInches + config_.lookaheadSpeedGain * currentSpeed,
                                                      config_.minLookaheadInches, maxLookaheadInches);

        searchIndex = findClosestIndexFrom(path, pose, searchIndex, maxLookaheadInches);
        const Waypoint lookaheadPoint = findLookaheadPoint(path, pose, searchIndex, lookaheadDistance);

        const LocalOffset local = toLocalFrame(pose, lookaheadPoint.x, lookaheadPoint.y);
        const double lSquared = local.forward * local.forward + local.strafeRight * local.strafeRight;
        const double curvature = lSquared > 1e-6 ? (2.0 * local.strafeRight / lSquared) : 0.0;

        const double distanceToEnd = distanceToPoint(pose, path.back().x, path.back().y);
        const double profiledSpeed = speedProfile.sample(elapsedSeconds).velocity;
        const double stoppingDistance = std::max(0.0, distanceToEnd - endToleranceInches);
        const double terminalSpeedCap =
            std::sqrt(2.0 * kTerminalDecelerationInchesPerSecondSquared * stoppingDistance);
        const bool useTerminalSpeed = distanceToEnd <= kTerminalControlDistanceInches ||
                                      elapsedSeconds >= speedProfile.getTotalDuration();
        const double remainingProfileSpeed = useTerminalSpeed ? kTerminalApproachSpeedInchesPerSecond : profiledSpeed;
        // The cap goes to zero AT the tolerance boundary, so on its own the
        // robot only ever approaches the boundary asymptotically -- slowing to
        // a speed too low to overcome friction just outside it, then sitting
        // there until timeout (seen 2026-11-19: stuck at 2.04in vs 2.00in
        // tolerance for 6.8s). Outside tolerance, never ask for less than a
        // speed that actually moves the robot; inside, the cap's zero stops it.
        double targetSpeed = std::min(remainingProfileSpeed, terminalSpeedCap);
        if (distanceToEnd > endToleranceInches) {
            targetSpeed = std::max(targetSpeed, kMinApproachSpeedInchesPerSecond);
        }

        const double signedTargetSpeed = backwards ? -targetSpeed : targetSpeed;
        const WheelSpeeds wheelSpeeds =
            curvatureToWheelSpeeds(signedTargetSpeed, curvature, config_.kinematics.trackWidthInches);
        drivetrain_.setTargetVelocity(inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics),
                                       inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics));

        settledCycles = distanceToEnd <= endToleranceInches ? settledCycles + 1 : 0;

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf("[purePursuit] pose=(%.1f, %.1f, %.1fdeg) lookahead=(%.1f, %.1f) curvature=%.4f speed=%.1f "
                        "endDistance=%.2f currentSpeed=%.1f%s\n",
                        pose.xInches, pose.yInches, pose.headingDegrees, lookaheadPoint.x, lookaheadPoint.y, curvature,
                        signedTargetSpeed, distanceToEnd, currentSpeed, backwards ? " backwards" : "");
            previousPrintTime = now;
        }

        const bool settled = settledCycles >= requiredSettleCycles;
        const bool timedOut = elapsedSeconds >= moveTimeoutSeconds;
        if (settled || timedOut) {
            std::printf("[purePursuit] %s after %.2fs, endpoint distance=%.2fin, tolerance=%.2fin\n",
                        settled ? "settled" : "timed out", elapsedSeconds, distanceToEnd, endToleranceInches);
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
    drivetrain_.coast();
}

}  // namespace lightspeed::motion
