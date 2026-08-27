#include "lightspeed/motion/drive_straight_distance.hpp"

#include <cstdio>

#include "lightspeed/motion/pose_math.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::motion {

DriveStraightDistance::DriveStraightDistance(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                                              const DriveStraightDistanceConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config), distancePidf_(config.distancePidf) {}

void DriveStraightDistance::run(double distanceInches) {
    distancePidf_.reset();

    const odom::Pose startPose = odometry_.getPose();
    const MotionProfile profile(0.0, distanceInches, config_.motionProfile);

    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;
    const double dtSeconds = config_.loopPeriodMs / 1000.0;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const odom::Pose currentPose = odometry_.getPose();
        const double traveledDistance = toLocalFrame(startPose, currentPose.xInches, currentPose.yInches).forward;

        const MotionState profileState = profile.sample(elapsedSeconds);
        const double targetForwardSpeed = distancePidf_.calculate(
            traveledDistance,
            control::Setpoint{
                .target = profileState.position, .targetVelocity = profileState.velocity, .targetAcceleration = profileState.acceleration},
            dtSeconds);

        const double headingError = headingErrorDegrees(currentPose.headingDegrees, startPose.headingDegrees);
        const double headingCorrection = config_.headingCorrectionKP * headingError;

        const WheelSpeeds wheelSpeeds =
            angularVelocityToWheelSpeeds(targetForwardSpeed, headingCorrection, config_.kinematics.trackWidthInches);
        drivetrain_.setTargetVelocity(inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics),
                                       inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics));

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf("[driveStraight] target=%.2fin actual=%.2fin heading=%.1fdeg (start %.1fdeg)\n",
                        profileState.position, traveledDistance, currentPose.headingDegrees, startPose.headingDegrees);
            previousPrintTime = now;
        }

        const bool profileFinished = elapsedSeconds >= profile.getTotalDuration();
        if ((profileFinished && distancePidf_.isSettled()) || elapsedSeconds >= config_.timeoutSeconds) {
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
}

}  // namespace lightspeed::motion
