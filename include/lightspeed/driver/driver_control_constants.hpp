/**
 * \file lightspeed/driver/driver_control_constants.hpp
 *
 * Single source of truth for driver control's tunable parameters. Adjust
 * here -- never inline in opcontrol().
 *
 * TODO: every value below is a placeholder pending real driver feel/testing.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/reference/configuration/
 */

#pragma once

#include "lightspeed/driver/accel_limit_resolver.hpp"
#include "lightspeed/driver/drive_mode.hpp"
#include "lightspeed/driver/input_profile.hpp"

namespace lightspeed::driver {

inline constexpr DriveMode kDriveMode = DriveMode::arcade;  // TODO: pick tank/arcade for the robot

inline const InputProfileConfig kInputProfileConfig{
    .curveExponent = 1.3,  // TODO: tune -- higher = softer low-speed response
    .deadband = 0.05,
};

// Blue 600 RPM motors; the external reduction is handled by drivetrain
// kinematics when converting linear speeds to motor RPM.
inline constexpr double kMaxDriveRpm = 600.0;

inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 12000.0,  // TODO: tune for driver feel
    .rules = {},
};

}  // namespace lightspeed::driver
