/**
 * \file lightspeed/odom/odometry_constants.hpp
 *
 * Single source of truth for the robot's odometry topology and conversion
 * factors. Adjust here -- never in the sources or fusion core themselves.
 */

#pragma once

#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

namespace detail {
inline constexpr double kPi = 3.14159265358979323846;
}  // namespace detail

inline constexpr double kTrackingWheelDiameterInches = 2.0;
inline constexpr double kTrackingWheelTicksToInches = (detail::kPi * kTrackingWheelDiameterInches) / 36000.0;

inline constexpr double kRobotWheelbaseInches = 10.0;
inline const TopologyConfig kCherenkovTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods =
        {
            {.name = "odometryPod",
             .role = PodRole::forward,
             .offsetInches = 0.0,
             .ticksToInches = kTrackingWheelTicksToInches},
        },
};

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 2.75,
    // Must match motion::kCherenkovKinematics.gearRatio -- measured ~1:1
    // (8.54in per motor rev), see the comment there.
    .gearRatio = 8.54 / (detail::kPi * 2.75),
};

}  // namespace lightspeed::odom
