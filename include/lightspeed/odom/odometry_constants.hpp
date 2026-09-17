/**
 * \file lightspeed/odom/odometry_constants.hpp
 *
<<<<<<< Updated upstream
 * Single source of truth for the robot's odometry topology and conversion
 * factors. Adjust here -- never in the sources or fusion core themselves.
 *
 * This robot has no tracking-wheel pods: odometry is IME (drive encoders) +
 * dual IMU only, so kOdometryTopology.pods is intentionally empty and the
 * kinematics fallback runs every cycle.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/odometry/
=======
 * Single source of truth for Cherenkov's odometry topology and conversion
 * factors. Adjust here -- never in the sources or fusion core themselves.
 *
 * TODO: every value below is a placeholder. This assumes a common 3-pod
 * tank layout (2 forward pods for redundant heading + 1 strafe pod), but
 * the resolver in OdometryFusion works for any 0-4 pod / role mix -- update
 * this file (and the matching ports in lightspeed::hal::config) once
 * Cherenkov's actual tracking-wheel hardware is built.
>>>>>>> Stashed changes
 */

#pragma once

#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

<<<<<<< Updated upstream
inline const TopologyConfig kOdometryTopology{
=======
namespace detail {
inline constexpr double kPi = 3.14159265358979323846;
}  // namespace detail

// Common small tracking wheel (NOT Cherenkov's 4in drive omnis). TODO: confirm.
inline constexpr double kTrackingWheelDiameterInches = 2.0;
inline constexpr double kTrackingWheelTicksToInches = (detail::kPi * kTrackingWheelDiameterInches) / 36000.0;

inline const TopologyConfig kCherenkovTopology{
>>>>>>> Stashed changes
    .kinematics = DrivetrainKinematics::tank,
    .pods = {},
};

inline const IMEConfig kDriveImeConfig{
<<<<<<< Updated upstream
    .wheelDiameterInches = 4.0,  // the robot's 4in drive omnis
=======
    .wheelDiameterInches = 4.0,  // Cherenkov's 4in drive omnis
>>>>>>> Stashed changes
    .gearRatio = 343.0 / 600.0,  // TODO: confirm external gear ratio; placeholder assumes blue (600rpm) cartridge geared to 343rpm output
};

}  // namespace lightspeed::odom
