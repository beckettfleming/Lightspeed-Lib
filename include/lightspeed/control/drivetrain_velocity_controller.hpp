/**
 * \file lightspeed/control/drivetrain_velocity_controller.hpp
 *
 * Voltage-based velocity control for one drivetrain (left + right side),
 * built from the generic PIDFController and SlewRateLimiter on top of the
 * Step 1 HAL motor group wrapper. Runs its own ~100Hz control task.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "lightspeed/control/pidf_controller.hpp"
#include "lightspeed/control/slew_rate_limiter.hpp"
#include "lightspeed/hal/motor_group.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::control {

struct DrivetrainVelocityConfig {
    PIDFConfig pidf;

    // Initial slew limit on the commanded voltage, in mV/s. Independently
    // swappable at runtime via each side's SlewRateLimiter if a future
    // caller needs to (this controller doesn't expose that itself, see
    // SlewRateLimiter::setMaxRate).
    double maxVoltageSlewRatePerSecond = 0.0;

    // Reference battery voltage (mV) that gains were tuned against. Output
    // is scaled by nominalBatteryMillivolts / actualBatteryMillivolts so
    // behavior doesn't drift as the pack sags.
    double nominalBatteryMillivolts = 12000.0;

    // Below this, battery-sag compensation stops boosting output further
    // (capped at 1.0x, not cut below it -- see updateSide()'s doc comment
    // for why this isn't a torque cutoff). Not season/robot-specific; V5
    // packs are broadly considered "getting low" around 11V, well below the
    // ~12.6-13.2V nominal full-charge range.
    double lowBatteryMillivolts = 11000.0;

    // Control task period. ~10ms (100Hz) matches the V5 motor's own
    // internal update rate -- running faster has no benefit.
    std::uint32_t loopPeriodMs = 10;
};

class DrivetrainVelocityController {
public:
    DrivetrainVelocityController(hal::MotorGroup& left, hal::MotorGroup& right, const DrivetrainVelocityConfig& config);
    ~DrivetrainVelocityController();

    // Owns a background control task referencing `this` -- not safe to copy
    // or move.
    DrivetrainVelocityController(const DrivetrainVelocityController&) = delete;
    DrivetrainVelocityController& operator=(const DrivetrainVelocityController&) = delete;

    // Sets the target velocity (RPM) for each side. Safe to call from any
    // task; the control task picks up the latest value each cycle.
    void setTargetVelocity(double leftRpm, double rightRpm);

private:
    void controlLoop();
    // isLeftSide picks which literal telemetry channel names this call
    // records under (see the .cpp) -- kept as a plain bool since it's only
    // ever used internally to select between two fixed name pairs, not
    // exposed anywhere else.
    void updateSide(hal::MotorGroup& motors, PIDFController& pidf, SlewRateLimiter& slew, double targetRpm, double dtSeconds,
                     bool isLeftSide) const;

    hal::MotorGroup& left_;
    hal::MotorGroup& right_;
    DrivetrainVelocityConfig config_;

    PIDFController leftPidf_;
    PIDFController rightPidf_;
    SlewRateLimiter leftSlew_;
    SlewRateLimiter rightSlew_;

    std::atomic<double> leftTargetRpm_{0.0};
    std::atomic<double> rightTargetRpm_{0.0};

    // Must be declared last: its initializer starts the control task, which
    // reads every other member above and must see them fully constructed.
    pros::Task task_;
};

}  // namespace lightspeed::control
