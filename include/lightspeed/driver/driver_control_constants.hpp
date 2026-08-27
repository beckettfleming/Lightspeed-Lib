/**
 * \file lightspeed/driver/driver_control_constants.hpp
 *
 * Single source of truth for driver control's tunable parameters. Adjust
 * here -- never inline in opcontrol() -- while running the bench harness in
 * src/main.cpp.
 *
 * TODO: every value below is a placeholder pending real driver feel/testing.
 */

#pragma once

#include "lightspeed/driver/accel_limit_resolver.hpp"
#include "lightspeed/driver/drive_mode.hpp"
#include "lightspeed/driver/input_profile.hpp"

namespace lightspeed::driver {

inline constexpr DriveMode kDriveMode = DriveMode::arcade;  // TODO: pick tank/arcade for Tachyon

inline const InputProfileConfig kInputProfileConfig{
    .curveExponent = 2.0,  // TODO: tune -- higher = softer low-speed response
    .deadband = 0.05,
};

// Matches the ~343 RPM top-speed reference already assumed by
// control::kDrivetrainVelocityConfig's kV (see that file) -- both should be
// confirmed/tuned together against Tachyon's actual drivetrain.
inline constexpr double kMaxDriveRpm = 343.0;

inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 2000.0,  // TODO: tune -- effectively "full send"
    .rules =
        {
            // Stand-in condition until a real subsystem flag exists: cap
            // acceleration harder while the demo arm reports extended, the
            // way a real lift/intake flag eventually would.
            AccelLimitRule{.flagName = "exampleArm.isExtended", .maxRpmPerSecond = 400.0},
        },
};

}  // namespace lightspeed::driver
