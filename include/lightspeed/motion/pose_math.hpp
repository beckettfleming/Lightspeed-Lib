/**
 * \file lightspeed/motion/pose_math.hpp
 *
 * Shared pose/geometry helpers for the motion primitives. Matches
 * lightspeed::odom's convention: heading in degrees, clockwise-positive; at
 * heading 0, local forward maps to field +y and strafe-right to field +x.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/reference/coordinates/
 */

#pragma once

#include "lightspeed/odom/types.hpp"

namespace lightspeed::motion {

struct LocalOffset {
    double forward;
    double strafeRight;
};

// Field-frame point -> robot local frame. Inverse of the local->field
// rotation OdometryFusion uses to integrate pose each cycle.
[[nodiscard]] LocalOffset toLocalFrame(const odom::Pose& robotPose, double targetX, double targetY);

[[nodiscard]] double distanceToPoint(const odom::Pose& robotPose, double targetX, double targetY);

// Absolute field heading toward a point, wrapped [0,360).
[[nodiscard]] double headingToPoint(const odom::Pose& robotPose, double targetX, double targetY);

// Shortest signed error to rotate from `from` to `to`, wrapped to
// (-180, 180]; positive = clockwise. Use this for EVERY heading comparison
// -- plain subtraction breaks across the 0/360 boundary.
[[nodiscard]] double headingErrorDegrees(double from, double to);

}  // namespace lightspeed::motion
