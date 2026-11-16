/**
 * \file lightspeed/telemetry/dashboard.hpp
 *
 * Live brain-screen dashboard for autonomous and driver control: battery,
 * pose, and confidence. Owns a background task gated on competition state;
 * the selector GUI owns the screen while disabled, so the two never contend
 * for pros::screen's mutex.
 *
 * Takes direct references rather than reading the telemetry bus: this is a
 * curated view of what a driver needs mid-match, not a generic dump.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/telemetry/
 */

#pragma once

#include <atomic>
#include <optional>

#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::telemetry {

class Dashboard {
public:
    Dashboard(const odom::OdometryFusion& odometry, const control::DrivetrainVelocityController& drivetrain,
              const std::atomic<bool>& limpModeEnabled);
    ~Dashboard();

    // Owns a background task referencing `this` -- not safe to copy or move.
    Dashboard(const Dashboard&) = delete;
    Dashboard& operator=(const Dashboard&) = delete;

    // Starts the background dashboard task. No-op if already started.
    void start();

private:
    void dashboardLoop();
    void draw() const;
    void drawDriveHealth() const;

    const odom::OdometryFusion& odometry_;
    const control::DrivetrainVelocityController& drivetrain_;
    const std::atomic<bool>& limpModeEnabled_;

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::telemetry
