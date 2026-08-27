#include "lightspeed/subsystem/demo/example_arm.hpp"

#include <cstdio>

namespace lightspeed::subsystem::demo {

namespace {

// Placeholder gains/presets/threshold -- tune on the bench harness, same as
// every other TODO-flagged constant in this project. Raw units are motor
// degrees (single motor, no external gearing assumed for this demo).
constexpr double kExtendedThresholdRawDegrees = 45.0;

const SubsystemConfig kExampleArmConfig{
    .name = "exampleArm",
    .pidf =
        {
            .kP = 15.0,
            .kI = 0.0,
            .kD = 1.0,
            .kV = 0.0,
            .kA = 0.0,
            .kS = 0.0,
            .integralZone = 0.0,
            .integralMax = 0.0,
            .settleTolerance = 5.0,  // degrees
            .settleCycles = 5,       // ~100ms at the scheduler's 50Hz
        },
    .presets =
        {
            Preset{.name = "LOW", .positionRaw = 0.0},
            Preset{.name = "MID", .positionRaw = 90.0},
            Preset{.name = "HIGH", .positionRaw = 180.0},
        },
};

}  // namespace

const char* toString(ExampleArmState state) {
    switch (state) {
        case ExampleArmState::idle:
            return "IDLE";
        case ExampleArmState::movingToTarget:
            return "MOVING_TO_TARGET";
        case ExampleArmState::holding:
            return "HOLDING";
        case ExampleArmState::faulted:
            return "FAULTED";
    }
    return "UNKNOWN";
}

ExampleArm::ExampleArm(hal::MotorGroup& motors) : Subsystem<ExampleArmState>(motors, kExampleArmConfig, ExampleArmState::idle) {
    registerFlag("exampleArm.isExtended", false);
}

void ExampleArm::onUpdate(double dtSeconds) {
    switch (getState()) {
        case ExampleArmState::idle:
            motors_.writeVoltage(0);
            break;
        case ExampleArmState::movingToTarget:
            driveToPosition(targetPositionRaw_, dtSeconds);
            if (pidf_.isSettled()) {
                transitionTo(ExampleArmState::holding);
            }
            break;
        case ExampleArmState::holding:
            driveToPosition(targetPositionRaw_, dtSeconds);
            break;
        case ExampleArmState::faulted:
            motors_.writeVoltage(0);
            if (motors_.getHealth() == hal::HealthStatus::ok) {
                transitionTo(ExampleArmState::idle);
            }
            break;
    }

    setFlag("exampleArm.isExtended", motors_.getPositionRaw() > kExtendedThresholdRawDegrees);
}

void ExampleArm::moveToPreset(const char* presetName) {
    const std::optional<double> position = getPresetPosition(presetName);
    if (!position.has_value()) {
        std::printf("[ExampleArm] WARNING: unknown preset '%s'\n", presetName);
        return;
    }

    // Atomic as a whole: transitionTo() re-enters the same recursive lock
    // on this task, so no scheduler cycle can observe the new target with
    // the old state or vice versa.
    lock();
    targetPositionRaw_ = *position;
    transitionTo(ExampleArmState::movingToTarget);
    unlock();
}

void ExampleArm::simulateFault() {
    lock();
    onFault(hal::HealthStatus::stalled);
    unlock();
}

void ExampleArm::onEnter(ExampleArmState state) {
    if (state == ExampleArmState::holding) {
        // Latch wherever we physically are right now as the hold target,
        // rather than trusting the target we were driving toward.
        targetPositionRaw_ = motors_.getPositionRaw();
    }
    std::printf("[ExampleArm] entering %s\n", toString(state));
}

void ExampleArm::onExit(ExampleArmState state) {
    std::printf("[ExampleArm] exiting %s\n", toString(state));
}

void ExampleArm::onFault(hal::HealthStatus status) {
    if (status == hal::HealthStatus::stalled && getState() != ExampleArmState::faulted) {
        std::printf("[ExampleArm] fault detected (%s) -- entering FAULTED\n", hal::toString(status));
        transitionTo(ExampleArmState::faulted);
    }
}

}  // namespace lightspeed::subsystem::demo
