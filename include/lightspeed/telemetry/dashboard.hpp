/**
 * \file lightspeed/telemetry/dashboard.hpp
 *
 * Live brain-screen dashboard for driver control: battery voltage, current
 * pose/confidence, subsystem state, and an active-fault indicator. Owns a
 * single background task, gated on competition state so it never draws
 * outside the driver-control period -- the Step 7 GUI selector owns the
 * screen through start-location/routine confirmation (disabled) and no one
 * draws during autonomous(), so the two never contend for pros::screen's
 * mutex (see the gating check in dashboardLoop()).
 *
 * Takes direct references to the objects it displays, the same pattern
 * AutonomousContext and SelectorGui already use -- a curated, hand-picked
 * subset of the robot's state, not a generic bus-driven view.
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
