#include "lightspeed/motion/move_to_pose.hpp"

#include <cmath>
#include <cstdio>

#include "lightspeed/motion/pose_math.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::motion {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
}  // namespace

MoveToPose::MoveToPose(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                        const MoveToPoseConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config) {}

void MoveToPose::run(double targetX, double targetY, double targetHeadingDegrees) {
    const odom::Pose startPose = odometry_.getPose();
    const double initialDistance = distanceToPoint(startPose, targetX, targetY);
    const MotionProfile speedProfile(0.0, initialDistance, config_.speedProfile);

    const double targetHeadingRadians = targetHeadingDegrees * kDegreesToRadians;

    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;
    std::uint32_t settledCycles = 0;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const odom::Pose pose = odometry_.getPose();
        const double distanceToTarget = distanceToPoint(pose, targetX, targetY);

        // Boomerang carrot: the target, walked back along ITS OWN heading
        // (not the robot's current one) by leadFraction * remaining
        // distance -- see the header doc for why this needs no separate
        // heading-blend term.
        const double carrotX = targetX - config_.leadFraction * distanceToTarget * std::sin(targetHeadingRadians);
        const double carrotY = targetY - config_.leadFraction * distanceToTarget * std::cos(targetHeadingRadians);

        const LocalOffset local = toLocalFrame(pose, carrotX, carrotY);
        const double lSquared = local.forward * local.forward + local.strafeRight * local.strafeRight;
        const double curvature = lSquared > 1e-6 ? (2.0 * local.strafeRight / lSquared) : 0.0;

        const double targetSpeed = speedProfile.sample(elapsedSeconds).velocity;

        const WheelSpeeds wheelSpeeds = curvatureToWheelSpeeds(targetSpeed, curvature, config_.kinematics.trackWidthInches);
        drivetrain_.setTargetVelocity(inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics),
                                       inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics));

        const double headingErrorMagnitude = std::abs(headingErrorDegrees(pose.headingDegrees, targetHeadingDegrees));
        const bool withinTolerance =
            distanceToTarget <= config_.positionToleranceInches && headingErrorMagnitude <= config_.headingToleranceDegrees;
        settledCycles = withinTolerance ? settledCycles + 1 : 0;

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf("[moveToPose] pose=(%.1f, %.1f, %.1fdeg) target=(%.1f, %.1f, %.1fdeg) dist=%.1f curvature=%.4f\n",
                        pose.xInches, pose.yInches, pose.headingDegrees, targetX, targetY, targetHeadingDegrees,
                        distanceToTarget, curvature);
            previousPrintTime = now;
        }

        if (settledCycles >= config_.settleCycles || elapsedSeconds >= config_.timeoutSeconds) {
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
}

}  // namespace lightspeed::motion
