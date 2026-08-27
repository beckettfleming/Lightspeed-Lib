/**
 * \file lightspeed/hal/config.hpp
 *
 * Logical-name -> physical V5 smart port map.
 *
 * This file must be the ONLY place a raw port number appears anywhere in the
 * codebase. Every subsystem should reference the named MotorGroupConfig
 * constants below (via lightspeed::hal::MotorGroup) instead of a literal
 * port number, so that rewiring the robot only ever means editing this file.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "pros/ai_vision.hpp"
#include "pros/motor_group.hpp"

namespace lightspeed::hal::config {

// V5 smart ports are numbered 1-21 on the Brain. A NEGATIVE port number
// tells pros::MotorGroup to reverse that motor internally, so every motor
// in a group can be commanded with the same sign of voltage and still turn
// the group's output shaft in the same physical direction.
//
// TODO: confirm actual port wiring once Tachyon's drivetrain is wired.
namespace port {

inline constexpr std::int8_t kLeftDriveFront = 1;
inline constexpr std::int8_t kLeftDriveMiddle = 2;
inline constexpr std::int8_t kLeftDriveBack = 3;
inline constexpr std::int8_t kRightDriveFront = -4;   // reversed
inline constexpr std::int8_t kRightDriveMiddle = -5;  // reversed
inline constexpr std::int8_t kRightDriveBack = -6;    // reversed

// Odometry tracking-wheel pods (Rotation sensors) and IMUs. Placeholder
// topology: 2 forward pods (left/right) + 1 strafe pod, dual IMU.
// TODO: confirm actual port wiring and final pod count once Tachyon's
// odometry hardware is built -- see lightspeed::odom for how these are
// consumed and how to add/remove pods.
inline constexpr std::int8_t kLeftForwardPodRotation = 7;
inline constexpr std::int8_t kRightForwardPodRotation = 8;
inline constexpr std::int8_t kStrafePodRotation = 9;
inline constexpr std::uint8_t kPrimaryImu = 10;
inline constexpr std::uint8_t kSecondaryImu = 11;

// DEMO/PLACEHOLDER -- lightspeed::subsystem::demo::ExampleArm validates the
// subsystem framework and is not a real Tachyon mechanism. Reuse or remove
// this port once real subsystems replace the demo.
inline constexpr std::int8_t kExampleArmMotor = 12;

// AI Vision Sensor (AprilTag final-approach correction, see
// lightspeed::vision). TODO: confirm actual port once mounted on Tachyon.
inline constexpr std::uint8_t kAiVisionSensor = 13;

}  // namespace port

// Describes one logical motor group: which ports back it, what gearing its
// motors use, and what units its encoders should report in. Add a new
// MotorGroupConfig here (plus ports in the `port` namespace above) whenever
// a new subsystem (lift, intake, ...) comes online.
struct MotorGroupConfig {
    const char* name;
    std::vector<std::int8_t> ports;
    pros::v5::MotorGears gearset;
    pros::v5::MotorUnits encoderUnits;
};

// TODO: confirm gearset once the drivetrain is built. Placeholder assumes
// blue (6:1, 600 RPM) cartridges with an external reduction bringing output
// to the ~343 RPM noted for Tachyon's drivetrain.
inline const MotorGroupConfig kLeftDriveGroup{
    "leftDriveGroup",
    {port::kLeftDriveFront, port::kLeftDriveMiddle, port::kLeftDriveBack},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kRightDriveGroup{
    "rightDriveGroup",
    {port::kRightDriveFront, port::kRightDriveMiddle, port::kRightDriveBack},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

// Describes one logical Rotation sensor or IMU: just a name + port, since
// (unlike motor groups) these HAL wrappers take no other construction-time
// configuration -- gearing/units concepts don't apply, and unit conversion
// (wheel diameter, etc.) happens in the odometry layer, not here.
struct RotationSensorConfig {
    const char* name;
    std::int8_t port;
};

struct ImuConfig {
    const char* name;
    std::uint8_t port;
};

inline const RotationSensorConfig kLeftForwardPodRotation{"leftForwardPodRotation", port::kLeftForwardPodRotation};
inline const RotationSensorConfig kRightForwardPodRotation{"rightForwardPodRotation", port::kRightForwardPodRotation};
inline const RotationSensorConfig kStrafePodRotation{"strafePodRotation", port::kStrafePodRotation};

inline const ImuConfig kPrimaryImu{"primaryImu", port::kPrimaryImu};
inline const ImuConfig kSecondaryImu{"secondaryImu", port::kSecondaryImu};

// DEMO/PLACEHOLDER -- backs lightspeed::subsystem::demo::ExampleArm, not a
// real Tachyon mechanism. See that class's header for why it exists.
inline const MotorGroupConfig kExampleArmGroup{
    "exampleArmGroup",
    {port::kExampleArmMotor},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

// Describes one logical AI Vision Sensor: name + port, plus which AprilTag
// family to detect (the sensor otherwise reports every family at once).
struct AiVisionConfig {
    const char* name;
    std::uint8_t port;
    pros::v5::AivisionTagFamily tagFamily;
};

// TODO: confirm the actual tag family used by the season's field elements
// once announced -- this is an arbitrary placeholder from the family list
// pros::v5::AivisionTagFamily exposes.
inline const AiVisionConfig kAiVisionSensor{"aiVisionSensor", port::kAiVisionSensor, pros::v5::AivisionTagFamily::tag_16H5};

}  // namespace lightspeed::hal::config
