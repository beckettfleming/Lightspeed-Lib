/**
 * \file lightspeed/hal/ai_vision_sensor.hpp
 *
 * Generic HAL wrapper around the VEX AI Vision Sensor, filtered to AprilTag
 * detections only (color/code/AI-object detection modes are never enabled
 * by this wrapper -- lightspeed::vision only needs tags). Exposes raw
 * per-tag corner pixel data; solving that into a bearing/distance/skew
 * reading is lightspeed::vision's job (see tag_pose_solver.hpp), not the
 * HAL layer's -- matches the project's existing HAL convention of staying
 * unit-agnostic/unconverted (see hal::RotationSensor's raw centidegrees,
 * hal::MotorGroup's raw encoder position).
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

    // Every currently detected AprilTag this cycle, filtered to
    // AivisionDetectType::tag (any color/code/object detections the sensor
    // might also report are skipped). Fixed-capacity, no allocation.
    [[nodiscard]] TagDetectionList getDetectedTags() const;

    [[nodiscard]] bool isHealthy() const;

private:
    // mutable (unlike hal::RotationSensor/Imu's sensor_ members): pros::AIVision's
    // get_object_count()/get_object()/is_installed() aren't const-qualified,
    // even though they're plain reads -- same reasoning as elsewhere in this
    // project for a read-only wrapper method calling a non-const PROS API.
    mutable pros::AIVision sensor_;
};

}  // namespace lightspeed::hal
