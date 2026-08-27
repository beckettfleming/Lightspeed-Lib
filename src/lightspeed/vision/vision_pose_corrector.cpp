#include "lightspeed/vision/vision_pose_corrector.hpp"

#include <cmath>
#include <limits>

#include "lightspeed/vision/tag_pose_solver.hpp"

namespace lightspeed::vision {

VisionPoseCorrector::VisionPoseCorrector(hal::AiVisionSensor& sensor, const CameraCalibration& calibration,
                                         const CameraMountOffset& mountOffset, const std::vector<TagWorldPose>& tagWorldMap,
                                         const VisionGatingConfig& gating)
    : sensor_(sensor), calibration_(calibration), mountOffset_(mountOffset), tagWorldMap_(tagWorldMap), gating_(gating) {}

const TagWorldPose* VisionPoseCorrector::findWorldPose(std::uint8_t tagId) const {
    for (const TagWorldPose& entry : tagWorldMap_) {
        if (entry.tagId == tagId) {
            return &entry;
        }
    }
    return nullptr;
}

VisionUpdate VisionPoseCorrector::update() {
    const hal::TagDetectionList detections = sensor_.getDetectedTags();

    // Prefer the closest detected tag that's actually in the world map
    // (closest = most reliable at the close range this feature targets).
    // If none of what's detected is in the map, fall back to the closest
    // detected tag at all, purely so the caller has something concrete to
    // report ("detected but unknown ID") instead of nothing.
    std::optional<hal::TagDetection> chosen;
    std::optional<RelativeTagReading> chosenRelative;
    const TagWorldPose* chosenWorldPose = nullptr;

    double bestKnownDistance = std::numeric_limits<double>::infinity();
    for (std::uint8_t i = 0; i < detections.count; ++i) {
        const TagWorldPose* worldPose = findWorldPose(detections.tags[i].id);
        if (worldPose == nullptr) {
            continue;
        }
        const RelativeTagReading reading = solveRelativePose(detections.tags[i], calibration_);
        if (reading.distanceInches < bestKnownDistance) {
            bestKnownDistance = reading.distanceInches;
            chosen = detections.tags[i];
            chosenRelative = reading;
            chosenWorldPose = worldPose;
        }
    }

    if (!chosen.has_value()) {
        double bestAnyDistance = std::numeric_limits<double>::infinity();
        for (std::uint8_t i = 0; i < detections.count; ++i) {
            const RelativeTagReading reading = solveRelativePose(detections.tags[i], calibration_);
            if (reading.distanceInches < bestAnyDistance) {
                bestAnyDistance = reading.distanceInches;
                chosen = detections.tags[i];
                chosenRelative = reading;
            }
        }
    }

    VisionUpdate result;

    if (!chosen.has_value()) {
        stableTagId_.reset();
        stableFrameCount_ = 0;
        result.reason = GateReason::noTagDetected;
        return result;
    }

    result.rawDetection = chosen;
    result.relative = *chosenRelative;

    if (chosenWorldPose == nullptr) {
        stableTagId_.reset();
        stableFrameCount_ = 0;
        result.reason = GateReason::unknownTagId;
        return result;
    }

    if (stableTagId_.has_value() && *stableTagId_ == chosen->id) {
        if (stableFrameCount_ < std::numeric_limits<std::uint8_t>::max()) {
            ++stableFrameCount_;
        }
    } else {
        stableTagId_ = chosen->id;
        stableFrameCount_ = 1;
    }
    result.stableFrameCount = stableFrameCount_;

    if (stableFrameCount_ < gating_.stableFrameThreshold) {
        result.reason = GateReason::notStableYet;
        return result;
    }

    if (std::abs(chosenRelative->skewDegrees) > gating_.maxSkewDegrees) {
        result.reason = GateReason::skewTooHigh;
        return result;
    }

    result.accepted = true;
    result.reason = GateReason::accepted;
    result.candidatePose = backOutRobotPose(*chosenRelative, chosenWorldPose->pose, mountOffset_);
    return result;
}

}  // namespace lightspeed::vision
