/**
 * \file lightspeed/odom/types.hpp
 *
 * Shared value types for the odometry subsystem: pod topology config, pose
 * and velocity, and the confidence tier the fusion core reports.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/odometry/
 */

#pragma once

#include <cstdint>
#include <vector>

namespace lightspeed::odom {

enum class PodRole : std::uint8_t { forward, strafe };

// This drivetrain is tank; holonomic has a stub fallback path in
// OdometryFusion, not a dead enumerator.
enum class DrivetrainKinematics : std::uint8_t { tank, holonomic };

// Declaration order IS confidence rank, best to worst -- which is what lets
// OdometryFusion::worseOf be a plain max().
enum class ConfidenceTier : std::uint8_t { fullPod = 0, partial = 1, imeOnly = 2 };

[[nodiscard]] const char* toString(ConfidenceTier tier);

struct PodConfig {
    const char* name;
    PodRole role;

    // Signed offset (inches) from the tracking center, measured along the
    // axis PERPENDICULAR to this pod's own rolling direction -- i.e. a
    // forward pod's offset is its left/right position, a strafe pod's
    // offset is its forward/backward position. This is the lever arm used
    // for rotation (chord) correction: positive = right (for a forward
    // pod) / forward (for a strafe pod). Sign convention should be
    // verified empirically on the bench harness -- flip it if a pure
    // in-place turn shows nonzero x/y drift.
    double offsetInches;

    // Raw-sensor-unit-to-inches conversion factor (centidegrees -> inches
    // for a Rotation-sensor-based pod: wheel circumference / 36000).
    double ticksToInches;
};

// 0-4 pods, any mix of roles, plus which kinematics to fall back to when an
// axis has no currently-healthy pods.
struct TopologyConfig {
    DrivetrainKinematics kinematics;
    std::vector<PodConfig> pods;
};

struct Pose {
    double xInches = 0.0;
    double yInches = 0.0;
    double headingDegrees = 0.0;  // clockwise-positive, wrapped to [0, 360)
};

struct Velocity {
    double xInchesPerSecond = 0.0;
    double yInchesPerSecond = 0.0;
    double headingDegreesPerSecond = 0.0;
};

}  // namespace lightspeed::odom
