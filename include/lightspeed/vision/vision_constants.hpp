/**
 * \file lightspeed/vision/vision_constants.hpp
 *
 * Single source of truth for every AprilTag-correction knob: camera
 * calibration, mounting offset, the tag-ID -> world-pose map, and gating
 * thresholds.
 *
 * TODO: every value below is a placeholder. None of it should be trusted
 * until measured against real hardware and the real season field via the
 * bench harness.
 *
 * Bringing vision online:
 * https://beckettfleming.github.io/Lightspeed-Lib/layers/vision/
 */

#pragma once

#include <cstdint>

#include "lightspeed/vision/vision_types.hpp"

namespace lightspeed::vision {

// TODO: replace with the AI Vision Sensor's actual calibrated focal
// length/principal point (from its factory calibration or a checkerboard
// calibration pass) and the robot's actual printed tag size once chosen.
// focalLengthPixels/principalPointXPixels below assume a 320px-wide frame
// and a rough guessed FOV -- placeholder only.
inline constexpr CameraCalibration kAiVisionCalibration{
    .focalLengthPixels = 460.0,
    .principalPointXPixels = 160.0,
    .tagSizeInches = 6.0,
};

// TODO: measure once the camera is actually mounted on the robot.
inline constexpr CameraMountOffset kAiVisionMountOffset{
    .xInches = 0.0,
    .yInches = 6.0,
    .headingDegreesOffset = 0.0,
};

// TODO: replace with actual season tag map once the field layout is
// published. Placeholder: two tags on opposite walls of a 144x144in field,
// facing inward.
inline const std::vector<TagWorldPose> kTagWorldMap{
    TagWorldPose{.tagId = 1, .pose = {.xInches = 0.0, .yInches = 72.0, .headingDegrees = 90.0}},
    TagWorldPose{.tagId = 2, .pose = {.xInches = 144.0, .yInches = 72.0, .headingDegrees = 270.0}},
};

// TODO: bench-tune stableFrameThreshold/maxSkewDegrees against a physically
// measured tag placement. correctionConfidence is high but deliberately not
// 1.0, so one noisy reading can't fully teleport the pose.
inline constexpr VisionGatingConfig kVisionGatingConfig{
    .stableFrameThreshold = 5,
    .maxSkewDegrees = 25.0,
    .correctionConfidence = 0.85,
};

// A print/observe loop, not a control loop -- well under the sensor's own
// frame rate.
inline constexpr std::uint32_t kVisionBenchTestLoopPeriodMs = 100;

}  // namespace lightspeed::vision
