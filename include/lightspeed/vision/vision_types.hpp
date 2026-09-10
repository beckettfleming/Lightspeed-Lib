/**
 * \file lightspeed/vision/vision_types.hpp
 *
 * Shared value types for AprilTag-based pose correction.
 *
 * Matches lightspeed::odom's convention: heading in degrees,
 * clockwise-positive; at heading 0, local forward maps to field +y and
 * strafe-right to field +x.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/vision/
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

// Fixed mounting offset from the robot's tracking center, in the robot's
// local frame (same convention as odom::PodConfig::offsetInches).
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

// Camera-relative, not yet combined with the tag's world pose.
struct RelativeTagReading {
    double bearingDegrees;   // camera optical-axis -> tag center, clockwise-positive
    double distanceInches;   // camera -> tag center
    double skewDegrees;      // tag face normal's yaw relative to "facing the camera squarely"; formula derived, sign needs on-hardware verification, see solver
};

struct VisionGatingConfig {
    std::uint8_t stableFrameThreshold;
    double maxSkewDegrees;
    double correctionConfidence;  // [0,1] blend weight; 1.0 == a full snap
};

enum class GateReason : std::uint8_t { accepted, noTagDetected, unknownTagId, notStableYet, skewTooHigh };

[[nodiscard]] const char* toString(GateReason reason);

}  // namespace lightspeed::vision
