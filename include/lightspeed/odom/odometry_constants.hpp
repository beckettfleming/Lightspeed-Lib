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
inline constexpr double kTrackingCenterOffsetInches = -1.5;  // behind center; sideways pod assumed at center

inline const TopologyConfig kCherenkovTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods =
        {
            {.name = "forwardPod",
             .role = PodRole::forward,
             .offsetInches = 0.0,
             .ticksToInches = kTrackingWheelTicksToInches},
            {.name = "sidewaysPod",
             .role = PodRole::strafe,
             .offsetInches = -kTrackingCenterOffsetInches,
             .ticksToInches = kTrackingWheelTicksToInches},
        },
};

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 2.75,
    .gearRatio = 3.0 / 4.0,
};

}  // namespace lightspeed::odom
