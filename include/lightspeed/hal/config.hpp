/**
 * \file lightspeed/hal/config.hpp
 *
 * Logical-name -> physical V5 smart port map. This file must be the ONLY
 * place a raw port number appears anywhere in the codebase.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/hal/
 */

#pragma once

#include <cstdint>
#include <vector>

#include "pros/ai_vision.hpp"
#include "pros/motor_group.hpp"

namespace lightspeed::hal::config {

// Ports are 1-21; a NEGATIVE port number reverses that motor internally.
//
// TODO: confirm actual port wiring once drivetrain is wired.
namespace port {

inline constexpr std::int8_t kLeftDriveFront = 1;
inline constexpr std::int8_t kLeftDriveRear = 9;
inline constexpr std::int8_t kRightDriveFront = -2;  // reversed
inline constexpr std::int8_t kRightDriveRear = -10;  // reversed
inline constexpr std::int8_t kIntakeFront = 4;
inline constexpr std::int8_t kIntakeRear = 5;

// Odometry: dual IMU only, no tracking-wheel pods -- see
// odom::kOdometryTopology (pods left empty) and lightspeed::odom for how
// IME + dual-IMU fusion works with zero pods configured.
// TODO: ports unknown -- confirm once the IMUs are actually mounted on
// the robot; these are unwired placeholders.
inline constexpr std::uint8_t kPrimaryImu = 3;
inline constexpr std::uint8_t kSecondaryImu = 8;

// DEMO/PLACEHOLDER -- lightspeed::subsystem::demo::ExampleArm validates the
// subsystem framework and is not a real mechanism. Reuse or remove
// this port once real subsystems replace the demo.
inline constexpr std::int8_t kExampleArmMotor = 12;

// AI Vision Sensor (AprilTag final-approach correction, see
// lightspeed::vision). TODO: confirm actual port once mounted on the robot.
inline constexpr std::uint8_t kAiVisionSensor = 13;

}  // namespace port

struct MotorGroupConfig {
    const char* name;
    std::vector<std::int8_t> ports;
    pros::v5::MotorGears gearset;
    pros::v5::MotorUnits encoderUnits;
};

// TODO: confirm gearset once the drivetrain is built. Placeholder assumes
// blue (6:1, 600 RPM) cartridges with an external reduction bringing output
// to the ~343 RPM target for the drivetrain.
inline const MotorGroupConfig kLeftDriveGroup{
    "leftDriveGroup",
    {port::kLeftDriveFront, port::kLeftDriveRear},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kRightDriveGroup{
    "rightDriveGroup",
    {port::kRightDriveFront, port::kRightDriveRear},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kIntakeFrontGroup{
    "intakeFrontGroup",
    {port::kIntakeFront},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kIntakeRearGroup{
    "intakeRearGroup",
    {port::kIntakeRear},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

struct RotationSensorConfig {
    const char* name;
    std::int8_t port;
};

struct ImuConfig {
    const char* name;
    std::uint8_t port;
};

inline const ImuConfig kPrimaryImu{"primaryImu", port::kPrimaryImu};
inline const ImuConfig kSecondaryImu{"secondaryImu", port::kSecondaryImu};

// DEMO/PLACEHOLDER -- backs lightspeed::subsystem::demo::ExampleArm.
inline const MotorGroupConfig kExampleArmGroup{
    "exampleArmGroup",
    {port::kExampleArmMotor},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

// tagFamily selects which AprilTag family to detect (the sensor otherwise
// reports every family at once).
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
