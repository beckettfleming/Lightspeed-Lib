/**
 * \file lightspeed/motion/motion_constants.hpp
 *
 * Single source of truth for the motion-control layer's tunable parameters.
 * Adjust here -- never inline in the primitives.
 *
 */

#pragma once

#include "lightspeed/motion/drive_straight_distance.hpp"
#include "lightspeed/motion/drivetrain_kinematics.hpp"
#include "lightspeed/motion/move_to_pose.hpp"
#include "lightspeed/motion/pure_pursuit_controller.hpp"
#include "lightspeed/motion/turn_to_heading.hpp"

namespace lightspeed::motion {

// The 3:4 external reduction turns the blue cartridge's 600 motor RPM into
// 450 wheel RPM.
inline const DrivetrainKinematicsConfig kCherenkovKinematics{
    .trackWidthInches = 275.0 / 25.4,
    .wheelDiameterInches = 2.75,
    .gearRatio = 3.0 / 4.0,
};

inline const TurnToHeadingConfig kTurnToHeadingConfig{
    .pidf =
        {
            .kP = 0.6,  // deg/s of angular velocity command per degree of heading error. TODO: tune
            .kI = 0.0,
            .kD = 0.05,
            .kV = 0.0,
            .kA = 0.0,
            .kS = 0.0,
            .integralZone = 0.0,
            .integralMax = 0.0,
            .settleTolerance = 2.0,  // degrees
            .settleCycles = 8,       // ~160ms at the 50Hz loop below
        },
    .timeoutSeconds = 3.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
};

inline const DriveStraightDistanceConfig kDriveStraightDistanceConfig{
    .distancePidf =
        {
            .kP = 3.0,  // (in/s of velocity command) per inch of position error. TODO: tune
            .kI = 0.0,
            .kD = 0.2,
            .kV = 1.0,  // pass the profile's own velocity straight through as feedforward
            .kA = 0.0,
            .kS = 0.0,
            .integralZone = 0.0,
            .integralMax = 0.0,
            .settleTolerance = 0.5,  // inches
            .settleCycles = 8,       // ~160ms at the 50Hz loop below
        },
    .motionProfile =
        {
            .maxVelocity = 40.0,      // in/s. TODO: tune
            .maxAcceleration = 80.0,  // in/s^2. TODO: tune
            .maxJerk = 400.0,         // in/s^3; S-curve on moves long enough to use it. TODO: tune
        },
    .headingCorrectionKP = 2.0,  // deg/s of correction per degree of drift. TODO: tune
    .timeoutSeconds = 5.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
};

inline const MoveToPoseConfig kMoveToPoseConfig{
    .leadFraction = 0.4,  // TODO: tune -- higher = earlier/more aggressive turn-in
    .positionToleranceInches = 2.0,
    .headingToleranceDegrees = 3.0,
    .settleCycles = 8,
    .timeoutSeconds = 6.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
    .speedProfile =
        {
            .maxVelocity = 40.0,
            .maxAcceleration = 80.0,
            .maxJerk = 400.0,
        },
};

inline const PurePursuitConfig kPurePursuitConfig{
    .minLookaheadInches = 6.0,   // TODO: tune
    .maxLookaheadInches = 18.0,  // TODO: tune
    .lookaheadSpeedGain = 0.3,   // additional inches of lookahead per in/s of current speed. TODO: tune
    .positionToleranceInches = 2.0,
    .settleCycles = 8,
    .timeoutSeconds = 8.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
    .speedProfile =
        {
            .maxVelocity = 40.0,
            .maxAcceleration = 80.0,
            .maxJerk = 400.0,
        },
    .smoothing =
        {
            .injectionSpacingInches = 2.0,
            .weightData = 0.5,
            .weightSmooth = 0.25,
            .smoothingToleranceInches = 0.001,
            .maxSmoothingIterations = 100,
        },
};

}  // namespace lightspeed::motion
