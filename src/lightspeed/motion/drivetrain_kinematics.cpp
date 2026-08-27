#include "lightspeed/motion/drivetrain_kinematics.hpp"

namespace lightspeed::motion {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
}  // namespace

double inchesPerSecondToRpm(double inchesPerSecond, const DrivetrainKinematicsConfig& config) {
    const double wheelRevsPerSecond = inchesPerSecond / (kPi * config.wheelDiameterInches);
    const double motorRevsPerSecond = wheelRevsPerSecond / config.gearRatio;
    return motorRevsPerSecond * 60.0;
}

double rpmToInchesPerSecond(double rpm, const DrivetrainKinematicsConfig& config) {
    const double motorRevsPerSecond = rpm / 60.0;
    const double wheelRevsPerSecond = motorRevsPerSecond * config.gearRatio;
    return wheelRevsPerSecond * kPi * config.wheelDiameterInches;
}

WheelSpeeds curvatureToWheelSpeeds(double forwardInchesPerSecond, double curvature, double trackWidthInches) {
    // Angular velocity = forward speed x curvature; substituting into the
    // differential-drive relation below gives the outer wheel (opposite
    // the curve direction) the larger share of speed.
    return WheelSpeeds{
        .leftInchesPerSecond = forwardInchesPerSecond * (1.0 + curvature * trackWidthInches / 2.0),
        .rightInchesPerSecond = forwardInchesPerSecond * (1.0 - curvature * trackWidthInches / 2.0),
    };
}

WheelSpeeds angularVelocityToWheelSpeeds(double forwardInchesPerSecond, double angularVelocityDegPerSecond,
                                          double trackWidthInches) {
    // Clockwise-positive angular velocity: left side forward / right side
    // back turns the robot clockwise (right), matching
    // lightspeed::odom's heading convention.
    const double angularVelocityRadPerSecond = angularVelocityDegPerSecond * kDegreesToRadians;
    const double wheelDelta = angularVelocityRadPerSecond * trackWidthInches / 2.0;
    return WheelSpeeds{
        .leftInchesPerSecond = forwardInchesPerSecond + wheelDelta,
        .rightInchesPerSecond = forwardInchesPerSecond - wheelDelta,
    };
}

}  // namespace lightspeed::motion
