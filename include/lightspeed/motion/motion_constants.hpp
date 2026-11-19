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

// gearRatio = wheel turns per motor turn. MEASURED, not derived: the drive is
// described as 2.75in wheels on a 36:48 pinion:spur (which would be 0.75), but
// three 12in runs on 2026-11-18 ([gearingCheck], tracking wheel vs motor
// encoders, odom confirmed against tape) gave 8.46-8.60in per motor rev --
// 1.31-1.33x the 6.48 that 0.75 implies, i.e. effectively ~1:1. With 0.75
// every commanded speed came out ~1.3x too fast, which was the root cause of
// the drive-straight overshoot. If the gearing is physically confirmed and
// differs, re-run the straight test and check [gearingCheck] ratio ~1.0.
inline constexpr double kMeasuredDriveInchesPerMotorRev = 8.54;
inline const DrivetrainKinematicsConfig kCherenkovKinematics{
    .trackWidthInches = 275.0 / 25.4,
    .wheelDiameterInches = 2.75,
    .gearRatio = kMeasuredDriveInchesPerMotorRev / (3.14159265358979323846 * 2.75),
};

inline const TurnToHeadingConfig kTurnToHeadingConfig{
    .pidf =
        {
            // deg/s of angular velocity command per degree of heading error.
            // Was 6 / kD 0.15, tuned while drive gearing was misconfigured (every
            // command came out 1.32x faster). After the gearing fix (2026-11-19)
            // turns stalled ~4deg short (~16rpm asked, below what overcomes
            // turning scrub) and ran to the 5s timeout. Scaled both by 1.32 to
            // restore the previously-tested behavior (~1deg, <1s).
            .kP = 8.0,
            // kI only acts inside integralZone: it builds push while the robot
            // sits stuck a couple of degrees short (2026-11-19: stalled at 2.4deg,
            // where kP alone asks ~19deg/s -- not enough to beat turning scrub)
            // until it breaks loose. integralMax caps that extra at 4*5 = 20deg/s.
            .kI = 6.0,  // was 4: broke loose but crept too slowly (up to ~2s)
            // Was 0.2: braked so hard on arrival that the robot slid back
            // 0.3-0.9deg (2026-11-19: 89.0->88.1, 177.1->176.7, 265.7->265.4),
            // then spent ~1s creeping back in on kI. 0.1 = gentler arrival.
            .kD = 0.1,
            .kV = 0.0,
            .kA = 0.0,
            .kS = 0.0,
            .integralZone = 5.0,  // degrees
            .integralMax = 5.0,   // degree-seconds
            // degrees. Was 2: with kI pushing, turns crept to just inside 2deg
            // and quit, landing consistently ~2deg short (2026-11-19).
            .settleTolerance = 1.0,
            .settleCycles = 8,       // ~160ms at the 50Hz loop below
        },
    .timeoutSeconds = 5.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
};

inline const DriveStraightDistanceConfig kDriveStraightDistanceConfig{
    .distancePidf =
        {
            // (in/s of velocity command) per inch of position error. Was 3: after
            // the gearing fix (2026-11-19) the robot trailed the profile by ~1.5in
            // at its end, and 3 in/s per inch left a ~1s, ~10rpm creep for the
            // last inch -- slow, and low enough to stick-slip (shudder). The
            // overshoot guard (maxBrakingDecel) caps this, so raising it can't
            // cause overshoot.
            .kP = 6.0,
            .kI = 0.0,
            .kD = 0.2,
            .kV = 1.0,  // pass the profile's own velocity straight through as feedforward
            // Lead term: (in/s) per (in/s^2) of profiled accel. On the ground the
            // wheels lag the commanded speed by ~0.15s (12in test 2026-11-18: 186rpm
            // asked, 99 measured at 0.25s; 13 asked, 61 measured during the stop),
            // so the robot fell behind, caught up, then overshot ~2in. kA pushes
            // harder while speeding up and brakes earlier while slowing. TODO: confirm
            .kA = 0.1,
            .kS = 0.0,
            .integralZone = 0.0,
            .integralMax = 0.0,
            .settleTolerance = 0.5,  // inches
            .settleCycles = 8,       // ~160ms at the 50Hz loop below
        },
    .motionProfile =
        {
            .maxVelocity = 40.0,      // in/s. TODO: tune
            .maxAcceleration = 60.0,  // in/s^2. Was 80: 12in test overshot ~7/8in then pulled back (2026-11-18). TODO: confirm
            .maxJerk = 400.0,         // in/s^3; S-curve on moves long enough to use it. TODO: tune
        },
    .headingCorrectionKP = 2.0,  // deg/s of correction per degree of drift. TODO: tune
    .timeoutSeconds = 5.0,
    .loopPeriodMs = 20,  // ~50Hz
    .kinematics = kCherenkovKinematics,
    // Overshoot guard (no-overshoot requirement: driving into stacks).
    // Deliberately gentler than the profile's 60 so the robot brakes on its
    // own measured position, not just the clock. Lag ~0.15s measured in the
    // 2026-11-18 12in tests. Still overshoots -> lower decel or raise lag.
    // Stops noticeably short / slow final creep -> raise decel. TODO: confirm
    .maxBrakingDecel = 40.0,
    .brakingLagSeconds = 0.15,
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
