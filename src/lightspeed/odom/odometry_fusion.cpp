#include "lightspeed/odom/odometry_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>

#include "lightspeed/telemetry/telemetry_bus.hpp"

namespace lightspeed::odom {

namespace {

constexpr std::uint32_t kLoopPeriodMs = 5;  // ~200Hz
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

double wrapDegrees(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped;
}

}  // namespace

OdometryFusion::OdometryFusion(DrivetrainKinematics kinematics, IMESource& leftIme, IMESource& rightIme, IMUSource& imu,
                                const std::vector<TrackingWheelSource*>& pods)
    : kinematics_(kinematics), leftIme_(leftIme), rightIme_(rightIme), imu_(imu) {
    for (TrackingWheelSource* pod : pods) {
        if (pod->getConfig().role == PodRole::forward) {
            forwardPods_[forwardPodCount_++] = pod;
        } else {
            strafePods_[strafePodCount_++] = pod;
        }
    }

    headingRedundancyAvailable_ = forwardPodCount_ >= 2 || strafePodCount_ >= 2;
    if (!headingRedundancyAvailable_) {
        std::printf(
            "[lightspeed::odom] WARNING: heading redundancy unavailable (no role has 2+ pods) -- "
            "IMU health is a hard dependency for heading in this topology.\n");
    }

    // Deferred-start: everything the task reads above must be fully set up
    // before it can safely run (see header comment on task_).
    task_.emplace([this] { fusionLoop(); }, "lightspeed_odometry_fusion");
}

OdometryFusion::~OdometryFusion() {
    task_->remove();
}

void OdometryFusion::fusionLoop() {
    std::uint32_t previousTime = pros::millis();
    constexpr double dtSeconds = kLoopPeriodMs / 1000.0;

    while (true) {
        pros::Task::delay_until(&previousTime, kLoopPeriodMs);

        resetMutex_.take();

        // Phase 1: unconditionally read every source so internal baselines
        // stay current even for sources that end up unused this cycle.
        const double leftImeDelta = leftIme_.readDeltaInches();
        const bool leftImeHealthy = leftIme_.isHealthy();
        const double rightImeDelta = rightIme_.readDeltaInches();
        const bool rightImeHealthy = rightIme_.isHealthy();
        const double imuHeadingDelta = imu_.readHeadingDeltaDegrees();
        const bool imuHealthy = imu_.isHealthy();

        DeltaArray forwardDeltas{};
        HealthArray forwardHealthy{};
        for (std::uint8_t i = 0; i < forwardPodCount_; ++i) {
            forwardDeltas[i] = forwardPods_[i]->readDeltaInches();
            forwardHealthy[i] = forwardPods_[i]->isHealthy();
        }
        DeltaArray strafeDeltas{};
        HealthArray strafeHealthy{};
        for (std::uint8_t i = 0; i < strafePodCount_; ++i) {
            strafeDeltas[i] = strafePods_[i]->readDeltaInches();
            strafeHealthy[i] = strafePods_[i]->isHealthy();
        }

        // Phase 2: resolve heading first -- the forward/strafe pod
        // correction below needs deltaTheta for each pod's lever-arm term.
        // IMU is always primary; pod-pair differential is fallback only.
        double headingDeltaDegrees = 0.0;
        if (imuHealthy) {
            headingDeltaDegrees = imuHeadingDelta;
        } else if (const auto differential =
                       differentialHeadingDegrees(forwardPods_, forwardDeltas, forwardHealthy, forwardPodCount_)) {
            headingDeltaDegrees = *differential;
        } else if (const auto differential =
                       differentialHeadingDegrees(strafePods_, strafeDeltas, strafeHealthy, strafePodCount_)) {
            headingDeltaDegrees = *differential;
        }
        // else: no heading source available this cycle -- heading holds.

        const double deltaThetaRadians = headingDeltaDegrees * kDegreesToRadians;

        std::uint8_t forwardHealthyCount = 0;
        double forwardTranslational =
            resolvePodAxis(forwardPods_, forwardDeltas, forwardHealthy, forwardPodCount_, deltaThetaRadians, forwardHealthyCount);
        if (forwardHealthyCount == 0) {
            forwardTranslational = resolveForwardFallback(leftImeDelta, rightImeDelta, leftImeHealthy, rightImeHealthy);
        }

        std::uint8_t strafeHealthyCount = 0;
        double strafeTranslational =
            resolvePodAxis(strafePods_, strafeDeltas, strafeHealthy, strafePodCount_, deltaThetaRadians, strafeHealthyCount);
        if (strafeHealthyCount == 0) {
            strafeTranslational = resolveStrafeFallback();
        }

        // Chord-length correction: converts the arc the tracking center
        // swept this cycle into the straight-line chord displacement that
        // belongs in x/y, converging to 1 as deltaTheta -> 0.
        const double chordFactor =
            (std::abs(deltaThetaRadians) < 1e-9) ? 1.0 : (2.0 * std::sin(deltaThetaRadians / 2.0) / deltaThetaRadians);
        const double forwardLocal = forwardTranslational * chordFactor;
        const double strafeLocal = strafeTranslational * chordFactor;

        // Rotate into the field frame using the midpoint heading (average
        // of heading before/after this cycle), not the stale old value.
        // Convention: heading clockwise-positive; at heading 0, local
        // forward maps to field +y and local strafe-right maps to +x.
        const double midpointHeadingRadians = (continuousHeadingDegrees_ + headingDeltaDegrees / 2.0) * kDegreesToRadians;
        const double dx = forwardLocal * std::sin(midpointHeadingRadians) + strafeLocal * std::cos(midpointHeadingRadians);
        const double dy = forwardLocal * std::cos(midpointHeadingRadians) - strafeLocal * std::sin(midpointHeadingRadians);

        continuousHeadingDegrees_ += headingDeltaDegrees;

        const ConfidenceTier confidence =
            worseOf(tierForAxis(forwardHealthyCount, forwardPodCount_), tierForAxis(strafeHealthyCount, strafePodCount_));

        Pose publishedPose{};
        Velocity publishedVelocity{};
        {
            auto lock = state_.lock();
            lock->pose.xInches += dx;
            lock->pose.yInches += dy;
            lock->pose.headingDegrees = wrapDegrees(continuousHeadingDegrees_);
            lock->velocity.xInchesPerSecond = dx / dtSeconds;
            lock->velocity.yInchesPerSecond = dy / dtSeconds;
            lock->velocity.headingDegreesPerSecond = headingDeltaDegrees / dtSeconds;
            lock->confidence = confidence;
            publishedPose = lock->pose;
            publishedVelocity = lock->velocity;
        }

        resetMutex_.give();

        // HAL wrapper health (motor-group health for the same physical
        // drive motors is recorded separately, see
        // DrivetrainVelocityController::updateSide) plus pose/velocity/
        // confidence -- everything OdometryFusion already computed this
        // cycle, published for the SD logger / dashboard to sample at
        // their own rate.
        telemetry::TelemetryBus& telemetryBus = telemetry::TelemetryBus::instance();
        telemetryBus.record("hal.leftIme.healthy", leftImeHealthy);
        telemetryBus.record("hal.rightIme.healthy", rightImeHealthy);
        telemetryBus.record("hal.imu.healthy", imuHealthy);
        telemetryBus.record("odom.forwardPods.healthyCount", static_cast<std::int32_t>(forwardHealthyCount));
        telemetryBus.record("odom.strafePods.healthyCount", static_cast<std::int32_t>(strafeHealthyCount));
        telemetryBus.record("odom.pose.x", publishedPose.xInches);
        telemetryBus.record("odom.pose.y", publishedPose.yInches);
        telemetryBus.record("odom.pose.heading", publishedPose.headingDegrees);
        telemetryBus.record("odom.velocity.x", publishedVelocity.xInchesPerSecond);
        telemetryBus.record("odom.velocity.y", publishedVelocity.yInchesPerSecond);
        telemetryBus.record("odom.velocity.headingRate", publishedVelocity.headingDegreesPerSecond);
        telemetryBus.record("odom.confidence", toString(confidence));
    }
}

double OdometryFusion::resolvePodAxis(const PodArray& pods, const DeltaArray& deltas, const HealthArray& healthy,
                                       std::uint8_t configuredCount, double deltaThetaRadians,
                                       std::uint8_t& outHealthyCount) const {
    DeltaArray corrected{};
    std::uint8_t healthyCount = 0;
    for (std::uint8_t i = 0; i < configuredCount; ++i) {
        if (!healthy[i]) {
            continue;
        }
        // Subtract the arc contributed purely by rotation about the
        // tracking center (this pod's lever arm), leaving the translational
        // component.
        corrected[healthyCount] = deltas[i] - pods[i]->getConfig().offsetInches * deltaThetaRadians;
        ++healthyCount;
    }
    outHealthyCount = healthyCount;

    if (healthyCount == 0) {
        return 0.0;
    }
    if (healthyCount == 1) {
        return corrected[0];
    }
    if (healthyCount == 2) {
        return (corrected[0] + corrected[1]) / 2.0;
    }

    // 3+: median, so one pod snagging on a field element gets outvoted
    // rather than dragging an average off. Pad unused slots with +infinity
    // and sort the whole fixed-size array (rather than sorting a
    // runtime-bounded slice of it) so the healthy values end up sorted in
    // the first healthyCount slots.
    for (std::uint8_t i = healthyCount; i < kMaxPodsPerRole; ++i) {
        corrected[i] = std::numeric_limits<double>::infinity();
    }
    std::sort(corrected.begin(), corrected.end());
    if (healthyCount % 2 == 1) {
        return corrected[healthyCount / 2];
    }
    return (corrected[healthyCount / 2 - 1] + corrected[healthyCount / 2]) / 2.0;
}

double OdometryFusion::resolveForwardFallback(double leftImeDelta, double rightImeDelta, bool leftHealthy,
                                               bool rightHealthy) const {
    if (kinematics_ != DrivetrainKinematics::tank) {
        // TODO: holonomic forward-IME fallback. Tachyon is tank and
        // doesn't need this path -- slot in the appropriate forward
        // combination here when a holonomic robot needs it.
        return 0.0;
    }
    if (leftHealthy && rightHealthy) {
        return (leftImeDelta + rightImeDelta) / 2.0;
    }
    if (leftHealthy) {
        return leftImeDelta;
    }
    if (rightHealthy) {
        return rightImeDelta;
    }
    return 0.0;  // both IME sides unhealthy -- no forward source available this cycle
}

double OdometryFusion::resolveStrafeFallback() const {
    if (kinematics_ == DrivetrainKinematics::tank) {
        // Tank drivetrains are structurally incapable of strafing -- always
        // exactly 0, no sensor needed.
        return 0.0;
    }
    // TODO: holonomic strafe fallback (derived from wheel kinematics).
    // Tachyon is tank and doesn't need this path.
    return 0.0;
}

std::optional<double> OdometryFusion::differentialHeadingDegrees(const PodArray& pods, const DeltaArray& deltas,
                                                                   const HealthArray& healthy,
                                                                   std::uint8_t configuredCount) {
    // Pick the healthy pair with the widest separation for the best
    // signal-to-noise ratio on the derived heading delta. "Any pair" per
    // the design brief -- widest is just the best choice among candidates.
    int bestA = -1;
    int bestB = -1;
    double bestSeparation = 0.0;
    for (std::uint8_t a = 0; a < configuredCount; ++a) {
        if (!healthy[a]) {
            continue;
        }
        for (std::uint8_t b = a + 1; b < configuredCount; ++b) {
            if (!healthy[b]) {
                continue;
            }
            const double separation = std::abs(pods[a]->getConfig().offsetInches - pods[b]->getConfig().offsetInches);
            if (separation > bestSeparation) {
                bestSeparation = separation;
                bestA = static_cast<int>(a);
                bestB = static_cast<int>(b);
            }
        }
    }

    if (bestA < 0) {
        return std::nullopt;
    }

    const double offsetDiff = pods[bestA]->getConfig().offsetInches - pods[bestB]->getConfig().offsetInches;
    const double deltaRadians = (deltas[bestA] - deltas[bestB]) / offsetDiff;
    return deltaRadians * kRadiansToDegrees;
}

ConfidenceTier OdometryFusion::tierForAxis(std::uint8_t healthyCount, std::uint8_t configuredCount) {
    if (configuredCount == 0 || healthyCount == 0) {
        return ConfidenceTier::imeOnly;
    }
    if (healthyCount == configuredCount) {
        return ConfidenceTier::fullPod;
    }
    return ConfidenceTier::partial;
}

ConfidenceTier OdometryFusion::worseOf(ConfidenceTier a, ConfidenceTier b) {
    // Declaration order fullPod < partial < imeOnly encodes rank, so a
    // plain max() picks the lower-confidence tier.
    return static_cast<ConfidenceTier>(std::max(static_cast<std::uint8_t>(a), static_cast<std::uint8_t>(b)));
}

Pose OdometryFusion::getPose() const {
    return state_.lock()->pose;
}

Velocity OdometryFusion::getVelocity() const {
    return state_.lock()->velocity;
}

ConfidenceTier OdometryFusion::getConfidence() const {
    return state_.lock()->confidence;
}

void OdometryFusion::applyVisionCorrection(const Pose& visionPose, double confidenceWeight) {
    const double weight = std::clamp(confidenceWeight, 0.0, 1.0);

    // Same atomicity guarantee setPose() relies on: resetMutex_ is held for
    // the entirety of a normal fusionLoop() cycle, so acquiring it here
    // blocks until any in-progress cycle finishes, then this correction is
    // fully applied before the next one starts.
    resetMutex_.take();

    {
        auto lock = state_.lock();

        // Shortest-path heading blend -- nudging by `weight` fraction of the
        // signed wrapped error, not a raw linear blend of the two heading
        // values (which would break across the 0/360 wrap boundary).
        double headingErrorDegrees = std::fmod(visionPose.headingDegrees - lock->pose.headingDegrees + 180.0, 360.0);
        if (headingErrorDegrees < 0.0) {
            headingErrorDegrees += 360.0;
        }
        headingErrorDegrees -= 180.0;

        lock->pose.xInches += weight * (visionPose.xInches - lock->pose.xInches);
        lock->pose.yInches += weight * (visionPose.yInches - lock->pose.yInches);
        lock->pose.headingDegrees = wrapDegrees(lock->pose.headingDegrees + weight * headingErrorDegrees);

        // continuousHeadingDegrees_ must move by the same delta as the
        // wrapped heading above -- otherwise the next fusionLoop() cycle's
        // `wrapDegrees(continuousHeadingDegrees_)` would silently overwrite
        // this correction right back to the pre-correction heading.
        continuousHeadingDegrees_ += weight * headingErrorDegrees;

        // velocity is left untouched -- a discrete position correction
        // doesn't imply the robot suddenly moved at some derived velocity.
    }

    resetMutex_.give();
}

void OdometryFusion::setPose(const Pose& pose) {
    resetMutex_.take();

    leftIme_.resetBaseline();
    rightIme_.resetBaseline();
    imu_.resetBaseline();
    for (std::uint8_t i = 0; i < forwardPodCount_; ++i) {
        forwardPods_[i]->resetBaseline();
    }
    for (std::uint8_t i = 0; i < strafePodCount_; ++i) {
        strafePods_[i]->resetBaseline();
    }

    continuousHeadingDegrees_ = pose.headingDegrees;

    {
        auto lock = state_.lock();
        lock->pose = pose;
        lock->velocity = Velocity{};
    }

    resetMutex_.give();
}

}  // namespace lightspeed::odom
