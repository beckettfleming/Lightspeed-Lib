#include "lightspeed/motion/drive_straight_distance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "lightspeed/motion/pose_math.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::motion {


DriveStraightDistance::DriveStraightDistance(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                                              const DriveStraightDistanceConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config), distancePidf_(config.distancePidf) {}

void DriveStraightDistance::run(double distanceInches, bool holdHeading) {
    run(distanceInches, holdHeading, config_.motionProfile);
}

void DriveStraightDistance::run(double distanceInches, bool holdHeading, const MotionProfileConfig& motionProfile,
                                bool useOvershootGuard, double timeoutSeconds) {
    const double moveTimeoutSeconds = timeoutSeconds > 0.0 ? timeoutSeconds : config_.timeoutSeconds;
    distancePidf_.reset();

    const odom::Pose startPose = odometry_.getPose();
    const MotionProfile profile(0.0, distanceInches, motionProfile);

    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;
    const double dtSeconds = config_.loopPeriodMs / 1000.0;
    constexpr double kSpeedFilterAlpha = 0.6;  // 1.0 = no smoothing
    double previousTraveledDistance = 0.0;
    double filteredForwardSpeed = 0.0;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const odom::Pose currentPose = odometry_.getPose();
        const double traveledDistance = toLocalFrame(startPose, currentPose.xInches, currentPose.yInches).forward;

        const MotionState profileState = profile.sample(elapsedSeconds);
        double targetForwardSpeed = distancePidf_.calculate(
            traveledDistance,
            control::Setpoint{
                .target = profileState.position, .targetVelocity = profileState.velocity, .targetAcceleration = profileState.acceleration},
            dtSeconds);

        // Overshoot guard -- see DriveStraightDistanceConfig::maxBrakingDecel.
        // Works in "toward the target" terms so it handles reverse moves too.
        // Only limits speed toward the target; backing off is never limited.
        // Speed is estimated here from this loop's own distance samples (20ms
        // apart, lightly smoothed) rather than odometry's per-cycle velocity,
        // which is a raw 10ms difference and can read near zero on cycles
        // where the tracking-wheel sensor hasn't produced a new sample --
        // exactly when the guard most needs a real number.
        const double rawForwardSpeed = (traveledDistance - previousTraveledDistance) / dtSeconds;
        previousTraveledDistance = traveledDistance;
        filteredForwardSpeed += kSpeedFilterAlpha * (rawForwardSpeed - filteredForwardSpeed);

        if (useOvershootGuard && config_.maxBrakingDecel > 0.0 && distanceInches != 0.0) {
            const double direction = distanceInches > 0.0 ? 1.0 : -1.0;
            const double speedTowardTarget = std::max(0.0, filteredForwardSpeed * direction);
            const double remaining =
                (distanceInches - traveledDistance) * direction - speedTowardTarget * config_.brakingLagSeconds;
            const double speedCap = remaining > 0.0 ? std::sqrt(2.0 * config_.maxBrakingDecel * remaining) : 0.0;
            if (targetForwardSpeed * direction > speedCap) {
                targetForwardSpeed = speedCap * direction;
            }
        }

        const double headingError = headingErrorDegrees(currentPose.headingDegrees, startPose.headingDegrees);
        const double headingCorrection = holdHeading ? config_.headingCorrectionKP * headingError : 0.0;

        const WheelSpeeds wheelSpeeds =
            angularVelocityToWheelSpeeds(targetForwardSpeed, headingCorrection, config_.kinematics.trackWidthInches);
        const double leftTargetRpm = inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics);
        const double rightTargetRpm = inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics);
        drivetrain_.setTargetVelocity(leftTargetRpm, rightTargetRpm);

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf(
                "[driveStraight] target=%.2fin actual=%.2fin heading=%.1fdeg (start %.1fdeg) "
                "error=%.1fdeg correction=%.1fdeg/s wheels target=(%.0f, %.0f)rpm measured=(%.0f, %.0f)rpm\n",
                profileState.position, traveledDistance, currentPose.headingDegrees, startPose.headingDegrees, headingError,
                headingCorrection, leftTargetRpm, rightTargetRpm, drivetrain_.getLeftMotorVelocityRpm(),
                drivetrain_.getRightMotorVelocityRpm());
            previousPrintTime = now;
        }

        const bool profileFinished = elapsedSeconds >= profile.getTotalDuration();
        if ((profileFinished && distancePidf_.isSettled()) || elapsedSeconds >= moveTimeoutSeconds) {
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
}

}  // namespace lightspeed::motion
