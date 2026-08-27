#include "lightspeed/control/drivetrain_velocity_controller.hpp"

#include <algorithm>

#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "pros/misc.hpp"

namespace lightspeed::control {

DrivetrainVelocityController::DrivetrainVelocityController(hal::MotorGroup& left, hal::MotorGroup& right,
                                                             const DrivetrainVelocityConfig& config)
    : left_(left),
      right_(right),
      config_(config),
      leftPidf_(config.pidf),
      rightPidf_(config.pidf),
      leftSlew_(config.maxVoltageSlewRatePerSecond),
      rightSlew_(config.maxVoltageSlewRatePerSecond),
      task_([this] { controlLoop(); }, "lightspeed_drivetrain_velocity") {}

DrivetrainVelocityController::~DrivetrainVelocityController() {
    task_.remove();
}

void DrivetrainVelocityController::setTargetVelocity(double leftRpm, double rightRpm) {
    leftTargetRpm_.store(leftRpm);
    rightTargetRpm_.store(rightRpm);
}

void DrivetrainVelocityController::controlLoop() {
    std::uint32_t previousTime = pros::millis();
    const double dtSeconds = static_cast<double>(config_.loopPeriodMs) / 1000.0;

    while (true) {
        pros::Task::delay_until(&previousTime, config_.loopPeriodMs);

        updateSide(left_, leftPidf_, leftSlew_, leftTargetRpm_.load(), dtSeconds, true);
        updateSide(right_, rightPidf_, rightSlew_, rightTargetRpm_.load(), dtSeconds, false);
    }
}

void DrivetrainVelocityController::updateSide(hal::MotorGroup& motors, PIDFController& pidf, SlewRateLimiter& slew,
                                               double targetRpm, double dtSeconds, bool isLeftSide) const {
    const double measuredRpm = motors.getVelocityRpm();
    const hal::HealthStatus health = motors.getHealth();

    const double batteryMv = static_cast<double>(pros::battery::get_voltage());
    // pros::battery::get_voltage() returns PROS_ERR (a large negative-cast
    // int) on a failed read, not just "0" -- treat anything non-positive as
    // an invalid read rather than a real 0V pack.
    const bool batteryReadValid = batteryMv > 0.0;
    const bool batteryLow = batteryReadValid && batteryMv < config_.lowBatteryMillivolts;

    telemetry::TelemetryBus& telemetryBus = telemetry::TelemetryBus::instance();
    if (isLeftSide) {
        telemetryBus.record("drivetrain.left.targetRpm", targetRpm);
        telemetryBus.record("drivetrain.left.actualRpm", measuredRpm);
        telemetryBus.record("drivetrain.left.health", hal::toString(health));
    } else {
        telemetryBus.record("drivetrain.right.targetRpm", targetRpm);
        telemetryBus.record("drivetrain.right.actualRpm", measuredRpm);
        telemetryBus.record("drivetrain.right.health", hal::toString(health));
    }
    telemetryBus.record("drivetrain.batteryLow", batteryLow);

    const std::uint8_t connectedMotors = motors.getConnectedMotorCount();
    telemetryBus.record(isLeftSide ? "drivetrain.left.connectedMotors" : "drivetrain.right.connectedMotors",
                         static_cast<std::int32_t>(connectedMotors));

    // Fail-safe: previously `health` was read and recorded for telemetry
    // only, and PIDF-computed voltage kept getting commanded every cycle
    // regardless of it -- a stalled/overheating/disconnected drive motor
    // would be driven indefinitely. `stalled`/`overTemperature` always stop
    // (continuing to drive through either is actively harmful, whether it's
    // 1 or all motors in the group); `disconnected` only stops the side if
    // EVERY motor in the group is gone -- a partial disconnect (getHealth()
    // collapses "1 of 3 down" and "3 of 3 down" into the same status, see
    // hal::MotorGroup::getHealth()'s doc comment) keeps driving in a
    // degraded state on whatever motors remain, rather than stopping a side
    // that's still partially drivable. Either way, zeroing also resets the
    // PIDF integrator and slew limiter so output ramps cleanly from zero
    // once health recovers, rather than resuming from stale state.
    const bool mustStop = health == hal::HealthStatus::stalled || health == hal::HealthStatus::overTemperature ||
                           (health == hal::HealthStatus::disconnected && connectedMotors == 0);
    if (mustStop) {
        pidf.reset();
        slew.reset(0.0);
        motors.writeVoltage(0);
        return;
    }

    // Velocity control: the feedback setpoint and the feedforward velocity
    // term are the same target RPM. No acceleration profile in this step,
    // so the feedforward acceleration term is always zero.
    const Setpoint setpoint{.target = targetRpm, .targetVelocity = targetRpm, .targetAcceleration = 0.0};
    const double rawOutputMv = pidf.calculate(measuredRpm, setpoint, dtSeconds);
    const double slewedOutputMv = slew.calculate(rawOutputMv, dtSeconds);

    // Battery-sag compensation boosts output as the pack sags, to keep
    // behavior consistent -- but boosting further once the pack is
    // critically low is counterproductive (accelerates the sag, risks a
    // brownout) rather than helpful, so the boost is capped at 1.0x (not
    // reduced below it) once batteryLow. This deliberately isn't a torque
    // cutoff: a critically low pack still needs to be driven off the field,
    // not stranded mid-match.
    const double compensationRatio =
        (batteryReadValid && !batteryLow) ? (config_.nominalBatteryMillivolts / batteryMv) : 1.0;
    const double compensatedMv = slewedOutputMv * compensationRatio;

    const double clampedMv = std::clamp(compensatedMv, -12000.0, 12000.0);
    motors.writeVoltage(static_cast<std::int32_t>(clampedMv));
}

}  // namespace lightspeed::control
