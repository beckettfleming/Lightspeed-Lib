/**
 * \file lightspeed/odom/odometry_constants.hpp
 *
 * Single source of truth for Tachyon's odometry topology and conversion
 * factors. Adjust here -- never in the sources or fusion core themselves.
 *
 * TODO: every value below is a placeholder. This assumes a common 3-pod
 * tank layout (2 forward pods for redundant heading + 1 strafe pod), but
 * the resolver in OdometryFusion works for any 0-4 pod / role mix -- update
 * this file (and the matching ports in lightspeed::hal::config) once
 * Tachyon's actual tracking-wheel hardware is built.
 */

#pragma once

#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

namespace detail {
inline constexpr double kPi = 3.14159265358979323846;
}  // namespace detail

// Common small tracking wheel (NOT Tachyon's 4in drive omnis). TODO: confirm.
inline constexpr double kTrackingWheelDiameterInches = 2.0;
inline constexpr double kTrackingWheelTicksToInches = (detail::kPi * kTrackingWheelDiameterInches) / 36000.0;

inline const TopologyConfig kTachyonTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods =
        {
            // Order matters: main.cpp pairs these with hal::RotationSensor
            // instances by index when building each TrackingWheelSource.
            PodConfig{
                .name = "leftForwardPod",
                .role = PodRole::forward,
                .offsetInches = 6.0,  // TODO: confirm -- left of tracking center
                .ticksToInches = kTrackingWheelTicksToInches,
            },
            PodConfig{
                .name = "rightForwardPod",
                .role = PodRole::forward,
                .offsetInches = -6.0,  // TODO: confirm -- right of tracking center
                .ticksToInches = kTrackingWheelTicksToInches,
            },
            PodConfig{
                .name = "strafePod",
                .role = PodRole::strafe,
                .offsetInches = 0.0,  // TODO: confirm -- forward/back offset from tracking center
                .ticksToInches = kTrackingWheelTicksToInches,
            },
        },
};

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 4.0,  // Tachyon's 4in drive omnis
    .gearRatio = 343.0 / 600.0,  // TODO: confirm external gear ratio; placeholder assumes blue (600rpm) cartridge geared to 343rpm output
};

}  // namespace lightspeed::odom
