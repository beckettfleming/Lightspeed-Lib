/**
 * \file lightspeed/motion/pose_math.hpp
 *
 * Shared pose/geometry helpers used across the motion primitives and pure
 * pursuit. Matches lightspeed::odom's convention exactly: heading in
 * degrees, clockwise-positive; at heading 0, local forward maps to field
 * +y and local strafe-right maps to field +x.
 */

#pragma once

#include "lightspeed/odom/types.hpp"

namespace lightspeed::motion {

struct LocalOffset {
    double forward;
    double strafeRight;
};

// Transforms a field-frame point into the robot's local frame (forward,
// strafe-right) given its current pose. Inverse of the local->field
// rotation OdometryFusion uses to integrate pose each cycle.
[[nodiscard]] LocalOffset toLocalFrame(const odom::Pose& robotPose, double targetX, double targetY);

// Straight-line distance from the robot's current position to a point.
[[nodiscard]] double distanceToPoint(const odom::Pose& robotPose, double targetX, double targetY);

// Absolute field heading (degrees, clockwise-positive, wrapped [0,360))
// from the robot's current position toward a point.
[[nodiscard]] double headingToPoint(const odom::Pose& robotPose, double targetX, double targetY);

// Shortest signed heading error (degrees) to rotate from `from` to `to`,
// wrapped to (-180, 180]. Positive = rotate clockwise.
[[nodiscard]] double headingErrorDegrees(double from, double to);

}  // namespace lightspeed::motion
