/**
 * \file lightspeed/subsystem/demo/example_arm.hpp
 *
 * ============================================================================
 * REFERENCE / PLACEHOLDER SUBSYSTEM -- NOT A REAL MECHANISM.
 *
 * Tachyon's actual mechanisms (lift, intake, etc.) aren't finalized. This
 * exists purely to exercise the subsystem framework end-to-end (state
 * machine, presets, PIDF position control, fault handling, flag registry)
 * on a single motor so the framework itself can be validated on hardware.
 * Delete or replace this once real subsystems are designed -- do not build
 * on top of it as if it were a real mechanism.
 * ============================================================================
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

    // Commands a transition to MovingToTarget, targeting the named preset
    // ("LOW" / "MID" / "HIGH"). Logs and does nothing if presetName isn't
    // registered. Safe to call from any task.
    void moveToPreset(const char* presetName);

    // Bench-test-only: forces the fault reaction without needing to
    // physically jam the mechanism. Not part of normal operation.
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
