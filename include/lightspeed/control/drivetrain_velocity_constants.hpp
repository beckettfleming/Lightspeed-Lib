/**
 * \file lightspeed/control/drivetrain_velocity_constants.hpp
 *
 * Single source of truth for the drivetrain velocity controller's tunable
 * parameters. Adjust gains here -- never in DrivetrainVelocityController.
 *
 * TODO: every gain below is a placeholder. Tune on the bench harness with
 * the robot's drive wheels off the ground.
 *
 * Tuning procedure:
 * https://beckettfleming.github.io/Lightspeed-Lib/guides/tuning/
 */

#pragma once

#include "lightspeed/control/drivetrain_velocity_controller.hpp"

namespace lightspeed::control {

inline const DrivetrainVelocityConfig kDrivetrainVelocityConfig{
    .pidf =
        {
            .kP = 20.0,  // mV per RPM of error. TODO: tune
            .kI = 0.0,   // TODO: tune once kP/kD are settled
            .kD = 0.0,   // TODO: tune
            .kV = 35.0,  // mV per target RPM, ~12000mV / 343rpm output. TODO: tune
            .kA = 0.0,   // unused until motion profiling exists
            .kS = 300.0, // mV to overcome static friction. TODO: tune
            .integralZone = 0.0,      // disabled until kI is tuned
            .integralMax = 4000.0,    // mV, hard windup clamp
            .settleTolerance = 10.0,  // RPM
            .settleCycles = 10,       // ~100ms at the 100Hz control rate
        },
    .maxVoltageSlewRatePerSecond = 240000.0,  // mV/s, full 12000mV swing in ~50ms. TODO: tune
    .nominalBatteryMillivolts = 12000.0,
    .lowBatteryMillivolts = 11000.0,  // generic V5-pack "getting low" threshold, not season-specific -- see field doc
    .loopPeriodMs = 10,  // ~100Hz, matches the V5 motor's own update rate
};

}  // namespace lightspeed::control
