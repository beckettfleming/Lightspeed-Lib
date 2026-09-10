/**
 * \file lightspeed/driver/drive_mode.hpp
 *
 * Converts already-profiled joystick input into normalized (-1..1)
 * left/right drivetrain output. Add a case to computeDriveOutput() for a new
 * mode without touching call sites.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/driver-control/
 */

#pragma once

namespace lightspeed::driver {

enum class DriveMode { tank, arcade, curvature };

struct DriveOutput {
    double left = 0.0;
    double right = 0.0;
};

struct JoystickInput {
    double leftY = 0.0;
    double leftX = 0.0;
    double rightY = 0.0;
    double rightX = 0.0;
};

// Each side driven directly by its own stick.
[[nodiscard]] DriveOutput tankDrive(double leftYInput, double rightYInput);

// Split arcade. Clamped per side, so turning at full forward can't exceed
// either side's own max.
[[nodiscard]] DriveOutput arcadeDrive(double forwardInput, double turnInput);

// Curvature ("cheesy") drive: turnInput's contribution scales with
// |forwardInput|. Below a small forward-magnitude threshold (see the .cpp)
// it falls back to a direct pivot -- without that, curvature steering alone
// can't turn from a standstill.
[[nodiscard]] DriveOutput curvatureDrive(double forwardInput, double turnInput);

// Input is expected to already be curve/deadband-profiled by the caller.
[[nodiscard]] DriveOutput computeDriveOutput(DriveMode mode, const JoystickInput& input);

}  // namespace lightspeed::driver
