/**
 * \file lightspeed/motion/drivetrain_kinematics.hpp
 *
 * Tank-drive kinematics + unit conversion shared by every motion primitive:
 * turning curvature/angular-velocity commands into left/right wheel
 * speeds, and wheel surface speed (inches/s) into the motor-shaft RPM units
 * lightspeed::control::DrivetrainVelocityController expects.
 */

#pragma once

namespace lightspeed::motion {

struct DrivetrainKinematicsConfig {
    double trackWidthInches;
    double wheelDiameterInches;
    double gearRatio;  // wheel revolutions per motor-output-shaft revolution (matches odom::IMEConfig convention)
};

struct WheelSpeeds {
    double leftInchesPerSecond;
    double rightInchesPerSecond;
};

[[nodiscard]] double inchesPerSecondToRpm(double inchesPerSecond, const DrivetrainKinematicsConfig& config);
[[nodiscard]] double rpmToInchesPerSecond(double rpm, const DrivetrainKinematicsConfig& config);

// Converts a forward speed + curvature (1/inches, e.g. pure pursuit's
// 2x/L^2, positive = curving right) into left/right wheel surface speeds.
[[nodiscard]] WheelSpeeds curvatureToWheelSpeeds(double forwardInchesPerSecond, double curvature, double trackWidthInches);

// Converts a forward speed + angular velocity (degrees/s,
// clockwise-positive, matching lightspeed::odom's heading convention) into
// left/right wheel surface speeds. Used by turn-to-heading (forward = 0)
// and drive-straight's heading-hold trim (angular velocity = small
// correction term).
[[nodiscard]] WheelSpeeds angularVelocityToWheelSpeeds(double forwardInchesPerSecond, double angularVelocityDegPerSecond,
                                                        double trackWidthInches);

}  // namespace lightspeed::motion
