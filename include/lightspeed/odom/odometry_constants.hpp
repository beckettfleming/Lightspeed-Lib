/**
 * \file lightspeed/odom/odometry_constants.hpp
 *
 * Single source of truth for the robot's odometry topology and conversion
 * factors. Adjust here -- never in the sources or fusion core themselves.
 *
 * This robot has no tracking-wheel pods: odometry is IME (drive encoders) +
 * dual IMU only. kOdometryTopology.pods is intentionally empty -- the
 * resolver in OdometryFusion falls back to drivetrain-kinematics-derived
 * forward/strafe whenever an axis has zero configured pods, which is always
 * true here. See lightspeed::hal::config for the (currently unwired) IMU
 * ports.
 */

#pragma once

#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

inline const TopologyConfig kOdometryTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods = {},
};

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 4.0,  // the robot's 4in drive omnis
    .gearRatio = 343.0 / 600.0,  // TODO: confirm external gear ratio; placeholder assumes blue (600rpm) cartridge geared to 343rpm output
};

}  // namespace lightspeed::odom
