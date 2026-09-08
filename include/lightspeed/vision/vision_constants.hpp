/**
 * \file lightspeed/vision/vision_constants.hpp
 *
 * Single source of truth for every AprilTag-correction knob: camera
 * calibration, mounting offset, the tag-ID -> world-pose map, and gating
 * thresholds.
 *
 * TODO: every value below is a placeholder -- see the comment on each
 * table. None of this should be trusted until it's been measured against
 * real hardware and the real season field via the bench harness (see
 * vision_bench_test.hpp).
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

// stableFrameThreshold/maxSkewDegrees: TODO tune on the bench harness
// against a physically measured tag placement. correctionConfidence is a
// blend weight (see OdometryFusion::applyVisionCorrection) -- high but not
// 1.0, so a single noisy reading can't fully teleport the pose; a run of
// gated-in readings converges on the true pose quickly at close range.
inline constexpr VisionGatingConfig kVisionGatingConfig{
    .stableFrameThreshold = 5,
    .maxSkewDegrees = 25.0,
    .correctionConfidence = 0.85,
};

// Bench harness diagnostic loop rate -- see vision_bench_test.hpp. Well
// under the AI Vision Sensor's own frame rate; this is a print/observe
// loop, not a control loop.
inline constexpr std::uint32_t kVisionBenchTestLoopPeriodMs = 100;

}  // namespace lightspeed::vision
