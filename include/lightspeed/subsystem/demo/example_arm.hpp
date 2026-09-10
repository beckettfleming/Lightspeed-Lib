/**
 * \file lightspeed/subsystem/demo/example_arm.hpp
 *
 * ============================================================================
 * REFERENCE / PLACEHOLDER SUBSYSTEM -- NOT A REAL MECHANISM.
 *
 * Exercises the subsystem framework end-to-end on a single motor. Delete or
 * replace once real subsystems are designed; do not build on top of it.
 * ============================================================================
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/subsystem/
 */

#pragma once

#include "lightspeed/hal/motor_group.hpp"
#include "lightspeed/subsystem/subsystem.hpp"

namespace lightspeed::subsystem::demo {

enum class ExampleArmState { idle, movingToTarget, holding, faulted };

[[nodiscard]] const char* toString(ExampleArmState state);

class ExampleArm : public Subsystem<ExampleArmState> {
public:
    explicit ExampleArm(hal::MotorGroup& motors);

    // "LOW" / "MID" / "HIGH". Logs and does nothing on an unknown name.
    // Safe from any task.
    void moveToPreset(const char* presetName);

    // Bench-test only: forces the fault reaction without physically jamming
    // the mechanism.
    void simulateFault();

protected:
    void onUpdate(double dtSeconds) override;
    void onEnter(ExampleArmState state) override;
    void onExit(ExampleArmState state) override;
    void onFault(hal::HealthStatus status) override;

private:
    // Guarded by Subsystem::lock()/unlock() whenever touched outside
    // onUpdate() (see moveToPreset()).
    double targetPositionRaw_ = 0.0;
};

}  // namespace lightspeed::subsystem::demo
