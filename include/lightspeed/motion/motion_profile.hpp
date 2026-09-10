/**
 * \file lightspeed/motion/motion_profile.hpp
 *
 * Generic 1D time-parameterized motion profile: trapezoidal, or jerk-limited
 * S-curve when config.maxJerk > 0 AND the move is long enough to reach both
 * max acceleration and max velocity with jerk-limited ramps (otherwise that
 * specific move falls back to trapezoidal). Unit-agnostic -- works in
 * whatever distance unit the caller passes.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/motion/
 */

#pragma once

namespace lightspeed::motion {

struct MotionProfileConfig {
    double maxVelocity;
    double maxAcceleration;
    double maxJerk = 0.0;  // <= 0 disables S-curve (pure trapezoidal)
};

struct MotionState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class MotionProfile {
public:
    MotionProfile(double startPosition, double endPosition, const MotionProfileConfig& config);

    // t is seconds since profile start, clamped to [0, getTotalDuration()].
    [[nodiscard]] MotionState sample(double timeSeconds) const;

    [[nodiscard]] double getTotalDuration() const {
        return totalDuration_;
    }

private:
    [[nodiscard]] MotionState accelBuildPhase(double tau) const;
    [[nodiscard]] MotionState sampleSCurve(double t) const;
    [[nodiscard]] MotionState sampleTrapezoid(double t) const;

    double startPosition_;
    double direction_;
    double distance_;
    double maxVelocity_;
    double maxAcceleration_;
    double maxJerk_;
    bool useSCurve_ = false;
    double totalDuration_ = 0.0;

    // Always computed as the fallback; used directly when useSCurve_ is false.
    double trapAccelTime_ = 0.0;
    double trapCruiseTime_ = 0.0;
    double trapPeakVelocity_ = 0.0;

    // S-curve segment params (only meaningful when useSCurve_ is true).
    double sJerkTime_ = 0.0;            // t1: duration of each jerk ramp
    double sConstAccelTime_ = 0.0;      // t2: duration at maxAcceleration_
    double sAccelPhaseDuration_ = 0.0;  // 2*t1 + t2
    double sCruiseTime_ = 0.0;
    double sV1_ = 0.0;   // velocity at the end of the first jerk ramp
    double sP1_ = 0.0;   // position at the end of the first jerk ramp
    double sVSeg2_ = 0.0;  // velocity at the end of the const-accel segment
    double sPSeg2_ = 0.0;  // position at the end of the const-accel segment
    double sAccelPhaseEndPos_ = 0.0;  // position once maxVelocity_ is reached
};

}  // namespace lightspeed::motion
