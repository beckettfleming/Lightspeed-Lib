/**
 * \file lightspeed/control/drivetrain_velocity_controller.hpp
 *
 * Voltage-based velocity control for one drivetrain (left + right side),
 * built from PIDFController and SlewRateLimiter on top of hal::MotorGroup.
 * Runs its own ~100Hz control task. Every drivetrain command in the project
 * -- driver control and every motion primitive -- funnels through
 * setTargetVelocity().
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/control/
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

    // Initial slew limit on the commanded voltage, in mV/s.
    double maxVoltageSlewRatePerSecond = 0.0;

    // Reference battery voltage (mV) the gains were tuned against. Output is
    // scaled by nominal/actual so behavior doesn't drift as the pack sags.
    double nominalBatteryMillivolts = 12000.0;

    // Below this, sag compensation stops boosting further (capped at 1.0x,
    // NOT cut below it -- a critically low pack still needs to drive off the
    // field).
    double lowBatteryMillivolts = 11000.0;

    // ~10ms (100Hz) matches the V5 motor's own internal update rate --
    // running faster has no benefit.
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

    // Target velocity (RPM) per side. Safe to call from any task; the
    // control task picks up the latest value each cycle.
    void setTargetVelocity(double leftRpm, double rightRpm);

private:
    void controlLoop();
    // isLeftSide selects which of two fixed telemetry channel-name pairs
    // this call records under.
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
