/**
 * \file lightspeed/vision/vision_types.hpp
 *
 * Shared value types for AprilTag-based pose correction: camera
 * calibration/mounting config, the tag-ID -> world-pose map, the relative
 * reading a single tag's corners solve to, and the gate-status enum the
 * bench harness (and any future caller) reports.
 *
 * Matches lightspeed::odom's convention exactly: heading in degrees,
 * clockwise-positive; at heading 0, local forward maps to field +y and
 * local strafe-right maps to field +x (see pose_math.hpp).
 */

#pragma once

#include <cstdint>
#include <vector>

#include "lightspeed/odom/types.hpp"

namespace lightspeed::vision {

struct CameraCalibration {
    double focalLengthPixels;
    double principalPointXPixels;
    double tagSizeInches;  // physical edge length of the printed tag
};

// Camera's fixed mounting offset from the robot's tracking center, in the
// robot's local frame (same convention as odom::PodConfig::offsetInches).
struct CameraMountOffset {
    double xInches;               // right of tracking center
    double yInches;                // forward of tracking center
    double headingDegreesOffset;   // camera optical-axis yaw relative to robot forward, clockwise-positive
};

// A known field AprilTag: pose.xInches/yInches is the tag's field position,
// pose.headingDegrees is the direction its face normal points outward,
// clockwise-positive (same convention as odom::Pose::headingDegrees).
struct TagWorldPose {
    std::uint8_t tagId;
    odom::Pose pose;
};

// A single tag's corners, solved into camera-relative bearing/distance/skew
// (see tag_pose_solver.hpp) -- not yet combined with the tag's world pose.
struct RelativeTagReading {
    double bearingDegrees;   // camera optical-axis -> tag center, clockwise-positive
    double distanceInches;   // camera -> tag center
    double skewDegrees;      // tag face normal's yaw relative to "facing the camera squarely"; formula derived, sign needs on-hardware verification, see solver
};

// Consecutive-frame stability threshold + skew rejection threshold + the
// blend weight a gated-in correction is applied with (see
// OdometryFusion::applyVisionCorrection).
struct VisionGatingConfig {
    std::uint8_t stableFrameThreshold;
    double maxSkewDegrees;
    double correctionConfidence;  // [0,1] blend weight; 1.0 == a full snap
};

enum class GateReason : std::uint8_t { accepted, noTagDetected, unknownTagId, notStableYet, skewTooHigh };

[[nodiscard]] const char* toString(GateReason reason);

}  // namespace lightspeed::vision
