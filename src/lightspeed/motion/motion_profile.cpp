#include "lightspeed/motion/motion_profile.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::motion {

MotionProfile::MotionProfile(double startPosition, double endPosition, const MotionProfileConfig& config)
    : startPosition_(startPosition),
      direction_(endPosition >= startPosition ? 1.0 : -1.0),
      distance_(std::abs(endPosition - startPosition)),
      maxVelocity_(config.maxVelocity),
      maxAcceleration_(config.maxAcceleration),
      maxJerk_(config.maxJerk) {
    if (distance_ <= 0.0) {
        totalDuration_ = 0.0;
        return;
    }

    // Try S-curve first if jerk-limiting was requested. t1 is the duration
    // of each jerk ramp (0->maxAcceleration_ or the reverse); t2 is the
    // duration spent at maxAcceleration_ once reached. If t2 comes out
    // negative, the move is too short to ever reach maxAcceleration_; if
    // the resulting accel-build phase alone (mirrored for decel) already
    // covers more than the whole move, it's too short to reach
    // maxVelocity_ either. Either case falls back to a plain trapezoidal
    // profile for this move rather than solving the shortened-S-curve
    // cases analytically.
    if (maxJerk_ > 0.0) {
        sJerkTime_ = maxAcceleration_ / maxJerk_;
        sV1_ = 0.5 * maxJerk_ * sJerkTime_ * sJerkTime_;
        sP1_ = maxJerk_ * sJerkTime_ * sJerkTime_ * sJerkTime_ / 6.0;
        sConstAccelTime_ = (maxVelocity_ - 2.0 * sV1_) / maxAcceleration_;

        if (sConstAccelTime_ >= 0.0) {
            sVSeg2_ = sV1_ + maxAcceleration_ * sConstAccelTime_;
            sPSeg2_ = sP1_ + sV1_ * sConstAccelTime_ + 0.5 * maxAcceleration_ * sConstAccelTime_ * sConstAccelTime_;
            sAccelPhaseEndPos_ = sPSeg2_ + sVSeg2_ * sJerkTime_ + 0.5 * maxAcceleration_ * sJerkTime_ * sJerkTime_ -
                                  maxJerk_ * sJerkTime_ * sJerkTime_ * sJerkTime_ / 6.0;
            sAccelPhaseDuration_ = 2.0 * sJerkTime_ + sConstAccelTime_;

            if (2.0 * sAccelPhaseEndPos_ <= distance_) {
                useSCurve_ = true;
                sCruiseTime_ = (distance_ - 2.0 * sAccelPhaseEndPos_) / maxVelocity_;
                totalDuration_ = 2.0 * sAccelPhaseDuration_ + sCruiseTime_;
            }
        }
    }

    if (!useSCurve_) {
        trapAccelTime_ = maxVelocity_ / maxAcceleration_;
        const double accelDist = 0.5 * maxAcceleration_ * trapAccelTime_ * trapAccelTime_;
        if (2.0 * accelDist > distance_) {
            // Triangular: never reaches maxVelocity_.
            trapPeakVelocity_ = std::sqrt(maxAcceleration_ * distance_);
            trapAccelTime_ = trapPeakVelocity_ / maxAcceleration_;
            trapCruiseTime_ = 0.0;
        } else {
            trapPeakVelocity_ = maxVelocity_;
            trapCruiseTime_ = (distance_ - 2.0 * accelDist) / maxVelocity_;
        }
        totalDuration_ = 2.0 * trapAccelTime_ + trapCruiseTime_;
    }
}

MotionState MotionProfile::accelBuildPhase(double tau) const {
    if (tau <= sJerkTime_) {
        return MotionState{
            .position = maxJerk_ * tau * tau * tau / 6.0,
            .velocity = 0.5 * maxJerk_ * tau * tau,
            .acceleration = maxJerk_ * tau,
        };
    }
    if (tau <= sJerkTime_ + sConstAccelTime_) {
        const double t = tau - sJerkTime_;
        return MotionState{
            .position = sP1_ + sV1_ * t + 0.5 * maxAcceleration_ * t * t,
            .velocity = sV1_ + maxAcceleration_ * t,
            .acceleration = maxAcceleration_,
        };
    }
    const double t = tau - sJerkTime_ - sConstAccelTime_;
    return MotionState{
        .position = sPSeg2_ + sVSeg2_ * t + 0.5 * maxAcceleration_ * t * t - maxJerk_ * t * t * t / 6.0,
        .velocity = sVSeg2_ + maxAcceleration_ * t - 0.5 * maxJerk_ * t * t,
        .acceleration = maxAcceleration_ - maxJerk_ * t,
    };
}

MotionState MotionProfile::sampleSCurve(double t) const {
    if (t <= sAccelPhaseDuration_) {
        return accelBuildPhase(t);
    }
    if (t <= sAccelPhaseDuration_ + sCruiseTime_) {
        const double dt = t - sAccelPhaseDuration_;
        return MotionState{
            .position = sAccelPhaseEndPos_ + maxVelocity_ * dt,
            .velocity = maxVelocity_,
            .acceleration = 0.0,
        };
    }
    const double timeFromEnd = totalDuration_ - t;
    const MotionState mirrored = accelBuildPhase(timeFromEnd);
    return MotionState{
        .position = distance_ - mirrored.position,
        .velocity = mirrored.velocity,
        .acceleration = -mirrored.acceleration,
    };
}

MotionState MotionProfile::sampleTrapezoid(double t) const {
    if (t <= trapAccelTime_) {
        return MotionState{
            .position = 0.5 * maxAcceleration_ * t * t,
            .velocity = maxAcceleration_ * t,
            .acceleration = maxAcceleration_,
        };
    }
    if (t <= trapAccelTime_ + trapCruiseTime_) {
        const double accelDist = 0.5 * maxAcceleration_ * trapAccelTime_ * trapAccelTime_;
        const double dt = t - trapAccelTime_;
        return MotionState{
            .position = accelDist + trapPeakVelocity_ * dt,
            .velocity = trapPeakVelocity_,
            .acceleration = 0.0,
        };
    }
    const double timeFromEnd = totalDuration_ - t;
    return MotionState{
        .position = distance_ - 0.5 * maxAcceleration_ * timeFromEnd * timeFromEnd,
        .velocity = maxAcceleration_ * timeFromEnd,
        .acceleration = -maxAcceleration_,
    };
}

MotionState MotionProfile::sample(double timeSeconds) const {
    if (distance_ <= 0.0) {
        return MotionState{.position = startPosition_, .velocity = 0.0, .acceleration = 0.0};
    }

    const double t = std::clamp(timeSeconds, 0.0, totalDuration_);
    const MotionState local = useSCurve_ ? sampleSCurve(t) : sampleTrapezoid(t);

    return MotionState{
        .position = startPosition_ + direction_ * local.position,
        .velocity = direction_ * local.velocity,
        .acceleration = direction_ * local.acceleration,
    };
}

}  // namespace lightspeed::motion
