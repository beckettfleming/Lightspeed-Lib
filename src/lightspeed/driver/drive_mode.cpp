#include "lightspeed/driver/drive_mode.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::driver {

namespace {
// Below this |forwardInput|, curvatureDrive() pivots directly on turnInput
// instead of scaling it toward zero -- small enough that it only kicks in
// right around a stationary stick, not during normal driving.
constexpr double kCurvatureQuickTurnThreshold = 0.05;
}  // namespace

DriveOutput tankDrive(double leftYInput, double rightYInput) {
    return DriveOutput{.left = leftYInput, .right = rightYInput};
}

DriveOutput arcadeDrive(double forwardInput, double turnInput) {
    return DriveOutput{
        .left = std::clamp(forwardInput + turnInput, -1.0, 1.0),
        .right = std::clamp(forwardInput - turnInput, -1.0, 1.0),
    };
}

DriveOutput curvatureDrive(double forwardInput, double turnInput) {
    if (std::abs(forwardInput) < kCurvatureQuickTurnThreshold) {
        return DriveOutput{
            .left = std::clamp(turnInput, -1.0, 1.0),
            .right = std::clamp(-turnInput, -1.0, 1.0),
        };
    }
    const double turnContribution = turnInput * std::abs(forwardInput);
    return DriveOutput{
        .left = std::clamp(forwardInput + turnContribution, -1.0, 1.0),
        .right = std::clamp(forwardInput - turnContribution, -1.0, 1.0),
    };
}

DriveOutput computeDriveOutput(DriveMode mode, const JoystickInput& input) {
    switch (mode) {
        case DriveMode::tank:
            return tankDrive(input.leftY, input.rightY);
        case DriveMode::arcade:
            return arcadeDrive(input.leftY, input.rightX);
        case DriveMode::curvature:
            return curvatureDrive(input.leftY, input.rightX);
    }
    return DriveOutput{};
}

}  // namespace lightspeed::driver
