#include "lightspeed/hal/ai_vision_sensor.hpp"

#include <algorithm>

namespace lightspeed::hal {

AiVisionSensor::AiVisionSensor(const config::AiVisionConfig& config) : sensor_(config.port) {
    sensor_.set_tag_family(config.tagFamily, true);
    sensor_.enable_detection_types(pros::AivisionModeType::tags);
}

TagDetectionList AiVisionSensor::getDetectedTags() const {
    TagDetectionList result;

    const std::int32_t objectCount = sensor_.get_object_count();
    if (objectCount <= 0) {
        return result;
    }

    const std::uint32_t clampedCount = std::min(static_cast<std::uint32_t>(objectCount),
                                                 static_cast<std::uint32_t>(kMaxTagDetections));
    for (std::uint32_t i = 0; i < clampedCount; ++i) {
        const pros::AIVision::Object object = sensor_.get_object(i);
        if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
            continue;
        }

        TagDetection& detection = result.tags[result.count];
        detection.id = object.id;
        detection.x0 = static_cast<double>(object.object.tag.x0);
        detection.y0 = static_cast<double>(object.object.tag.y0);
        detection.x1 = static_cast<double>(object.object.tag.x1);
        detection.y1 = static_cast<double>(object.object.tag.y1);
        detection.x2 = static_cast<double>(object.object.tag.x2);
        detection.y2 = static_cast<double>(object.object.tag.y2);
        detection.x3 = static_cast<double>(object.object.tag.x3);
        detection.y3 = static_cast<double>(object.object.tag.y3);
        ++result.count;
    }

    return result;
}

bool AiVisionSensor::isHealthy() const {
    return sensor_.is_installed();
}

}  // namespace lightspeed::hal
