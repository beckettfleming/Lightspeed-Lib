/**
 * \file lightspeed/driver/drive_mode.hpp
 *
 * Converts already-profiled joystick input into normalized (-1..1)
 * left/right drivetrain output. Kept as a swappable transform rather than
 * hardcoded to one scheme -- add a case to computeDriveOutput() for a new
 * mode (e.g. curvature) without touching call sites.
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

// Tank: each side driven directly by its own stick.
[[nodiscard]] DriveOutput tankDrive(double leftYInput, double rightYInput);

// Split arcade: forwardInput drives both sides together, turnInput
// differentially steers. Output is clamped to [-1, 1] per side, so turning
// at full forward speed can't exceed either side's own max.
[[nodiscard]] DriveOutput arcadeDrive(double forwardInput, double turnInput);

// Curvature ("cheesy") drive: turnInput's contribution scales with
// |forwardInput|, so small turn corrections stay gentle at high speed
// instead of always applying full differential steering the way arcade
// does. Below a small forward-magnitude threshold, falls back to a direct
// tank-style pivot (turnInput drives each side oppositely) so the robot can
// still rotate in place -- without that, curvature steering alone can't
// turn from a standstill (turn's contribution scales toward zero as
// forward does). See the .cpp for the threshold.
[[nodiscard]] DriveOutput curvatureDrive(double forwardInput, double turnInput);

// Dispatches to the selected mode, using whichever axes of input it needs
// (input is expected to already be curve/deadband-profiled by the caller).
[[nodiscard]] DriveOutput computeDriveOutput(DriveMode mode, const JoystickInput& input);

}  // namespace lightspeed::driver
