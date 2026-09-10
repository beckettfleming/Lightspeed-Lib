/**
 * \file lightspeed/vision/vision_pose_corrector.hpp
 *
 * Reads the AI Vision HAL wrapper, solves each detected tag's corners, picks
 * the best candidate, tracks frame-to-frame stability, and gates on skew.
 * update() never touches OdometryFusion itself -- it reports a diagnostic
 * result and the caller decides whether to apply it.
 *
 * Deliberately not a full PnP solve: close-range drift correction doesn't
 * need 6DOF, and the corner-ratio approximation is far cheaper per cycle.
 * Corrections are trust-gated rather than continuous, since vision is slower
 * and higher-latency than the 200Hz fusion core.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/vision/
 */

#pragma once

#include <cstdint>
#include <optional>

#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/vision/vision_types.hpp"

namespace lightspeed::vision {

struct VisionUpdate {
    bool accepted = false;
    GateReason reason = GateReason::noTagDetected;

    // Populated even when REJECTED, so the bench harness can show why.
    std::optional<hal::TagDetection> rawDetection;
    RelativeTagReading relative{};        // valid only if rawDetection has a value
    std::uint8_t stableFrameCount = 0;    // consecutive frames of the same ID, capped at 255

    odom::Pose candidatePose{};  // valid only if accepted
};

class VisionPoseCorrector {
public:
    VisionPoseCorrector(hal::AiVisionSensor& sensor, const CameraCalibration& calibration,
                         const CameraMountOffset& mountOffset, const std::vector<TagWorldPose>& tagWorldMap,
                         const VisionGatingConfig& gating);

    // One full cycle. Picks the closest KNOWN tag, or -- purely for
    // diagnostics -- the closest detected tag when none are known.
    [[nodiscard]] VisionUpdate update();

private:
    [[nodiscard]] const TagWorldPose* findWorldPose(std::uint8_t tagId) const;

    hal::AiVisionSensor& sensor_;
    const CameraCalibration& calibration_;
    const CameraMountOffset& mountOffset_;
    const std::vector<TagWorldPose>& tagWorldMap_;
    const VisionGatingConfig& gating_;

    std::optional<std::uint8_t> stableTagId_;
    std::uint8_t stableFrameCount_ = 0;
};

}  // namespace lightspeed::vision
