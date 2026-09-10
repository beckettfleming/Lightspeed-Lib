/**
 * \file lightspeed/odom/ime_source.hpp
 *
 * Forward-distance source backed by a drive motor group's built-in encoders.
 * Tank's zero-pod forward fallback; construct one per drive side so the
 * fusion core can average left/right.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/odometry/
 */

#pragma once

#include "lightspeed/hal/motor_group.hpp"

namespace lightspeed::odom {

struct IMEConfig {
    double wheelDiameterInches;
    double gearRatio;  // wheel revolutions per motor-output-shaft revolution
};

class IMESource {
public:
    IMESource(hal::MotorGroup& motors, const IMEConfig& config);

    // Inches since the last read. Zero if unhealthy or on the first read
    // after construction/resetBaseline().
    double readDeltaInches();

    // A stalled or over-temperature motor still reports a valid encoder
    // position -- only an actually disconnected group invalidates this.
    [[nodiscard]] bool isHealthy() const;

    void resetBaseline();

private:
    hal::MotorGroup& motors_;
    IMEConfig config_;
    double previousPositionDeg_ = 0.0;
    bool hasBaseline_ = false;
};

}  // namespace lightspeed::odom
