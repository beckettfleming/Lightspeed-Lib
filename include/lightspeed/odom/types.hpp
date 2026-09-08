/**
 * \file lightspeed/odom/types.hpp
 *
 * Shared value types for the odometry subsystem: pod topology config, pose
 * and velocity, and the confidence tier the fusion core reports alongside
 * them.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace lightspeed::odom {

enum class PodRole : std::uint8_t { forward, strafe };

// This drivetrain is tank. The field exists so the topology config and
// resolver logic don't assume a specific drivetrain -- Holonomic is a
// recognized value with a stub fallback path (see OdometryFusion), not a dead
// enumerator.
enum class DrivetrainKinematics : std::uint8_t { tank, holonomic };

// Coarse trust level for the current pose, based on how much of the
// configured tracking-wheel topology is actually healthy right now.
// Declaration order is also confidence rank (best to worst) -- see
// OdometryFusion::worseOf.
enum class ConfidenceTier : std::uint8_t { fullPod = 0, partial = 1, imeOnly = 2 };

[[nodiscard]] const char* toString(ConfidenceTier tier);

// Describes one tracking-wheel pod's geometry and role. Pure config -- no
// port number and no hal::RotationSensor reference here (those live in
// hal::config and are paired with a PodConfig only where a
// TrackingWheelSource is constructed), so this stays reusable wherever a
// pod's geometry needs describing.
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

// The full pod topology for a robot: 0-4 pods, any mix of roles, plus which
// kinematics to use when an axis has no currently-healthy pods.
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
