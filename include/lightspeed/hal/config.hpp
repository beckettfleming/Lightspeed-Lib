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
/* Drive Port Mapping
RF 1
RM 2
RR 3
LF 4
LM 5
LR 6
*/

namespace port {

inline constexpr std::int8_t kRightDriveFront = 1;
inline constexpr std::int8_t kRightDriveMiddle = 2;
inline constexpr std::int8_t kRightDriveRear = 3;
inline constexpr std::int8_t kLeftDriveFront = 4;
inline constexpr std::int8_t kLeftDriveMiddle = 5;
inline constexpr std::int8_t kLeftDriveRear = 6;

inline constexpr std::int8_t kForwardPodRotation = 7;
inline constexpr std::int8_t kSidewaysPodRotation = 8;

inline constexpr std::uint8_t kPrimaryImu = 9;
inline constexpr std::uint8_t kSecondaryImu = 10;
inline constexpr std::int8_t kWinchLeft = 11;
inline constexpr std::int8_t kIntake = -12;
inline constexpr std::int8_t kWinchRight = 13;

}  // namespace port

struct MotorGroupConfig {
    const char* name;
    std::vector<std::int8_t> ports;
    pros::v5::MotorGears gearset;
    pros::v5::MotorUnits encoderUnits;
};

inline const MotorGroupConfig kLeftDriveGroup{
    "leftDriveGroup",
    {port::kLeftDriveFront, port::kLeftDriveMiddle, port::kLeftDriveRear},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kRightDriveGroup{
    "rightDriveGroup",
    {port::kRightDriveFront, port::kRightDriveMiddle, port::kRightDriveRear},
    pros::v5::MotorGears::blue,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kWinchLeftGroup{
    "winchLeftGroup",
    {port::kWinchLeft},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kWinchRightGroup{
    "winchRightGroup",
    {port::kWinchRight},
    pros::v5::MotorGears::green,
    pros::v5::MotorUnits::degrees,
};

inline const MotorGroupConfig kIntakeGroup{
    "intakeGroup",
    {port::kIntake},
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

struct AiVisionConfig {
    const char* name;
    std::uint8_t port;
    pros::v5::AivisionTagFamily tagFamily;
};

inline const ImuConfig kPrimaryImu{"primaryImu", port::kPrimaryImu};
inline const ImuConfig kSecondaryImu{"secondaryImu", port::kSecondaryImu};

inline const RotationSensorConfig kForwardPodRotation{"forwardPodRotation", port::kForwardPodRotation};
inline const RotationSensorConfig kSidewaysPodRotation{"sidewaysPodRotation", port::kSidewaysPodRotation};

}  // namespace lightspeed::hal::config
