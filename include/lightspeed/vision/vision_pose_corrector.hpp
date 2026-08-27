/**
 * \file lightspeed/vision/vision_pose_corrector.hpp
 *
 * Ties the pieces together: reads the AI Vision HAL wrapper, solves each
 * detected tag's corners (tag_pose_solver.hpp), picks the best candidate,
 * tracks its frame-to-frame stability, and gates it against the configured
 * skew threshold. update() never touches OdometryFusion itself -- it just
 * reports a full diagnostic result; the caller (the bench harness, or any
 * future live caller) decides whether/how to feed an accepted result into
 * OdometryFusion::applyVisionCorrection().
 *
 * Deliberately not a full PnP solve (see tag_pose_solver.hpp): correcting
 * drift on close-range final approach doesn't need a precise 6DOF solve,
 * and the corner-ratio approximation this project uses instead is far
 * cheaper to run every cycle. Corrections are also trust-gated rather than
 * continuous -- vision updates are slower/higher-latency than the 200Hz
 * fusion core, so this only fires a discrete correction once a reading is
 * actually trustworthy, rather than blending in every cycle.
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

    // The tag this cycle's result is about, if any were detected at all
    // (populated even when rejected, so the bench harness can show why).
    std::optional<hal::TagDetection> rawDetection;
    RelativeTagReading relative{};        // meaningful only if rawDetection has a value
    std::uint8_t stableFrameCount = 0;    // consecutive frames the same ID has been seen, capped at 255

    odom::Pose candidatePose{};  // meaningful only if accepted
};

class VisionPoseCorrector {
public:
    VisionPoseCorrector(hal::AiVisionSensor& sensor, const CameraCalibration& calibration,
                         const CameraMountOffset& mountOffset, const std::vector<TagWorldPose>& tagWorldMap,
                         const VisionGatingConfig& gating);

    // Runs one full cycle: reads the sensor, solves + gates the best
    // candidate tag (closest known tag if any were detected, otherwise the
    // closest detected tag at all purely for diagnostics -- see the .cpp).
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
