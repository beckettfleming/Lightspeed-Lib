#include "lightspeed/motion/turn_to_heading.hpp"

#include <cmath>
#include <cstdio>

#include "lightspeed/motion/pose_math.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::motion {

TurnToHeading::TurnToHeading(control::DrivetrainVelocityController& drivetrain, odom::OdometryFusion& odometry,
                              const TurnToHeadingConfig& config)
    : drivetrain_(drivetrain), odometry_(odometry), config_(config), pidf_(config.pidf) {}

void TurnToHeading::run(double targetHeadingDegrees) {
    pidf_.reset();

    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    std::uint32_t previousPrintTime = startTime;
    constexpr std::uint32_t kPrintIntervalMs = 250;
    const double dtSeconds = config_.loopPeriodMs / 1000.0;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);
        const std::uint32_t now = pros::millis();
        const double elapsedSeconds = (now - startTime) / 1000.0;

        const double currentHeading = odometry_.getPose().headingDegrees;
        const double wrappedError = headingErrorDegrees(currentHeading, targetHeadingDegrees);

        // PIDF is fed measurement=0 and target=wrappedError each cycle
        // (recomputed fresh every time), so its P/I/D terms always act on
        // the correct shortest-path error rather than naively differencing
        // raw headings across the 0/360 wrap boundary.
        const double angularVelocityDegPerSecond = pidf_.calculate(
            0.0, control::Setpoint{.target = wrappedError, .targetVelocity = 0.0, .targetAcceleration = 0.0}, dtSeconds);

        const WheelSpeeds wheelSpeeds =
            angularVelocityToWheelSpeeds(0.0, angularVelocityDegPerSecond, config_.kinematics.trackWidthInches);
        drivetrain_.setTargetVelocity(inchesPerSecondToRpm(wheelSpeeds.leftInchesPerSecond, config_.kinematics),
                                       inchesPerSecondToRpm(wheelSpeeds.rightInchesPerSecond, config_.kinematics));

        if (now - previousPrintTime >= kPrintIntervalMs) {
            std::printf("[turnToHeading] target=%.1fdeg actual=%.1fdeg error=%.1fdeg\n", targetHeadingDegrees,
                        currentHeading, wrappedError);
            previousPrintTime = now;
        }

        if (pidf_.isSettled() || elapsedSeconds >= config_.timeoutSeconds) {
            break;
        }
    }

    drivetrain_.setTargetVelocity(0.0, 0.0);
}

}  // namespace lightspeed::motion
