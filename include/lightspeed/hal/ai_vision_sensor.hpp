/**
 * \file lightspeed/hal/ai_vision_sensor.hpp
 *
 * Generic HAL wrapper around the VEX AI Vision Sensor, filtered to AprilTag
 * detections only (color/code/AI-object modes are never enabled here).
 * Exposes raw per-tag corner pixel data; solving that into a
 * bearing/distance/skew reading is lightspeed::vision's job.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/hal/
 */

#pragma once

#include <array>
#include <cstdint>

#include "lightspeed/hal/config.hpp"
#include "pros/ai_vision.hpp"

namespace lightspeed::hal {

// Pixel corners of one detected AprilTag, in whatever order the sensor
// reports them (see tag_pose_solver.hpp for how corner0/3 and corner1/2
// pair up into the tag's left/right edges).
struct TagDetection {
    std::uint8_t id;
    double x0, y0;
    double x1, y1;
    double x2, y2;
    double x3, y3;
};

// Hardware's own hard cap on simultaneously detected objects (of any type,
// not just tags) -- see AIVISION_MAX_OBJECT_COUNT.
inline constexpr std::uint8_t kMaxTagDetections = 24;

struct TagDetectionList {
    std::array<TagDetection, kMaxTagDetections> tags{};
    std::uint8_t count = 0;
};

class AiVisionSensor {
public:
    explicit AiVisionSensor(const config::AiVisionConfig& config);

    // Fixed-capacity, no allocation.
    [[nodiscard]] TagDetectionList getDetectedTags() const;

    [[nodiscard]] bool isHealthy() const;

private:
    // mutable: pros::AIVision's get_object_count()/get_object()/
    // is_installed() aren't const-qualified even though they're plain reads.
    mutable pros::AIVision sensor_;
};

}  // namespace lightspeed::hal
