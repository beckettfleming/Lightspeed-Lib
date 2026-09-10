/**
 * \file lightspeed/motion/drivetrain_kinematics.hpp
 *
 * Tank-drive kinematics + unit conversion shared by every motion primitive.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
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

// curvature is 1/inches, positive = curving right.
[[nodiscard]] WheelSpeeds curvatureToWheelSpeeds(double forwardInchesPerSecond, double curvature, double trackWidthInches);

// Angular velocity is degrees/s, clockwise-positive. Used by turn-to-heading
// (forward = 0) and drive-straight's heading-hold trim.
[[nodiscard]] WheelSpeeds angularVelocityToWheelSpeeds(double forwardInchesPerSecond, double angularVelocityDegPerSecond,
                                                        double trackWidthInches);

}  // namespace lightspeed::motion
