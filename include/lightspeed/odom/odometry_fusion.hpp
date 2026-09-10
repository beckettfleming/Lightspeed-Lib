/**
 * \file lightspeed/odom/odometry_fusion.hpp
 *
 * Fuses IME, IMU, and 0-4 tracking-wheel pods (any role distribution) into
 * a single field-frame pose, running its own ~200Hz task. The per-cycle
 * algorithm lives in odometry_fusion.cpp.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/odometry/
 */

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/imu_source.hpp"
#include "lightspeed/odom/tracking_wheel_source.hpp"
#include "lightspeed/odom/types.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::odom {

// Worst case is every configured pod sharing one role (e.g. "all-forward"),
// so this bounds each role's array, not the whole-robot pod count.
inline constexpr std::uint8_t kMaxPodsPerRole = 4;

class OdometryFusion {
public:
    // pods: 0-4, any mix of roles; the pointers must outlive this object.
    // kinematics selects the forward/strafe fallback used whenever an axis
    // has zero currently-healthy pods.
    OdometryFusion(DrivetrainKinematics kinematics, IMESource& leftIme, IMESource& rightIme, IMUSource& imu,
                   const std::vector<TrackingWheelSource*>& pods);
    ~OdometryFusion();

    // Owns a background fusion task referencing `this` -- not safe to copy
    // or move.
    OdometryFusion(const OdometryFusion&) = delete;
    OdometryFusion& operator=(const OdometryFusion&) = delete;

    [[nodiscard]] Pose getPose() const;
    [[nodiscard]] Velocity getVelocity() const;
    [[nodiscard]] ConfidenceTier getConfidence() const;

    // Resets pose AND every source's delta baseline in one step, atomic
    // with respect to the fusion task, so the next cycle can't see a
    // spurious jump from comparing an old baseline to a fresh pose.
    void setPose(const Pose& pose);

    // Weighted blend toward visionPose; confidenceWeight in [0,1], 1.0 being
    // a full snap. Unlike setPose(), does NOT reset source baselines -- this
    // corrects a running estimate rather than re-initializing it, so
    // in-flight deltas since the last cycle are preserved.
    void applyVisionCorrection(const Pose& visionPose, double confidenceWeight);

private:
    struct FusedState {
        Pose pose;
        Velocity velocity;
        ConfidenceTier confidence = ConfidenceTier::imeOnly;
    };

    using PodArray = std::array<TrackingWheelSource*, kMaxPodsPerRole>;
    using DeltaArray = std::array<double, kMaxPodsPerRole>;
    using HealthArray = std::array<bool, kMaxPodsPerRole>;

    void fusionLoop();

    // Averages (2 healthy pods) or takes the median (3+) of the healthy,
    // lever-arm-corrected pod deltas for one axis. outHealthyCount == 0
    // means the caller must apply the drivetrain-kinematics fallback.
    [[nodiscard]] double resolvePodAxis(const PodArray& pods, const DeltaArray& deltas, const HealthArray& healthy,
                                         std::uint8_t configuredCount, double deltaThetaRadians,
                                         std::uint8_t& outHealthyCount) const;

    [[nodiscard]] double resolveForwardFallback(double leftImeDelta, double rightImeDelta, bool leftHealthy,
                                                 bool rightHealthy) const;
    [[nodiscard]] double resolveStrafeFallback() const;

    // Differential heading (degrees) from the widest-spaced healthy pair
    // sharing a role, or nullopt if no such pair is currently healthy.
    [[nodiscard]] static std::optional<double> differentialHeadingDegrees(const PodArray& pods, const DeltaArray& deltas,
                                                                           const HealthArray& healthy,
                                                                           std::uint8_t configuredCount);

    [[nodiscard]] static ConfidenceTier tierForAxis(std::uint8_t healthyCount, std::uint8_t configuredCount);
    [[nodiscard]] static ConfidenceTier worseOf(ConfidenceTier a, ConfidenceTier b);

    DrivetrainKinematics kinematics_;
    IMESource& leftIme_;
    IMESource& rightIme_;
    IMUSource& imu_;

    PodArray forwardPods_{};
    std::uint8_t forwardPodCount_ = 0;
    PodArray strafePods_{};
    std::uint8_t strafePodCount_ = 0;

    // From pod *configuration*, not runtime health: true if any role has 2+
    // configured pods, i.e. a differential-heading fallback pair could ever
    // exist in this topology.
    bool headingRedundancyAvailable_ = false;

    double continuousHeadingDegrees_ = 0.0;

    mutable pros::MutexVar<FusedState> state_;
    pros::Mutex resetMutex_;

    // Deferred-start: constructed at the end of the constructor body, once
    // pod partitioning above is complete, so the task never observes
    // partially set-up state (see odometry_fusion.cpp).
    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::odom
