/**
 * \file lightspeed/odom/tracking_wheel_source.hpp
 *
 * Distance source backed by one Rotation-sensor tracking-wheel pod. 0-4 of
 * these, in any mix of PodRole, feed the fusion core's per-axis resolution.
 */

#pragma once

#include "lightspeed/hal/rotation_sensor.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::odom {

class TrackingWheelSource {
public:
    TrackingWheelSource(hal::RotationSensor& sensor, const PodConfig& config);

    // Distance delta (inches) since the last read, via config.ticksToInches.
    // Zero if unhealthy or on the first read after
    // construction/resetBaseline().
    double readDeltaInches();

    [[nodiscard]] bool isHealthy() const;

    void resetBaseline();

    [[nodiscard]] const PodConfig& getConfig() const {
        return config_;
    }

private:
    hal::RotationSensor& sensor_;
    PodConfig config_;
    double previousPositionCentidegrees_ = 0.0;
    bool hasBaseline_ = false;
};

}  // namespace lightspeed::odom
