/**
 * \file lightspeed/telemetry/dashboard.hpp
 *
 * Live brain-screen dashboard for driver control: battery, pose, confidence,
 * subsystem state, and a fault indicator. Owns a background task gated on
 * competition state so it never draws outside driver control -- the selector
 * GUI owns the screen before the match and nothing draws during autonomous,
 * so the two never contend for pros::screen's mutex.
 *
 * Takes direct references rather than reading the telemetry bus: this is a
 * curated view of what a driver needs mid-match, not a generic dump.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/telemetry/
 */

#pragma once

#include <optional>

#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/subsystem/demo/example_arm.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::telemetry {

class Dashboard {
public:
    Dashboard(const odom::OdometryFusion& odometry, const subsystem::demo::ExampleArm& exampleArm);
    ~Dashboard();

    // Owns a background task referencing `this` -- not safe to copy or move.
    Dashboard(const Dashboard&) = delete;
    Dashboard& operator=(const Dashboard&) = delete;

    // Starts the background dashboard task. No-op if already started.
    void start();

private:
    void dashboardLoop();
    void draw() const;

    const odom::OdometryFusion& odometry_;
    const subsystem::demo::ExampleArm& exampleArm_;

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::telemetry
