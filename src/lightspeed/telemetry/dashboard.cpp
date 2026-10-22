#include "lightspeed/telemetry/dashboard.hpp"

#include "lightspeed/telemetry/dashboard_layout.hpp"
#include "lightspeed/telemetry/telemetry_constants.hpp"
#include "pros/misc.hpp"
#include "pros/screen.hpp"

namespace lightspeed::telemetry {

Dashboard::Dashboard(const odom::OdometryFusion& odometry) : odometry_(odometry) {}

Dashboard::~Dashboard() {
    if (task_.has_value()) {
        task_->remove();
    }
}

void Dashboard::start() {
    if (task_.has_value()) {
        return;
    }
    task_.emplace([this] { dashboardLoop(); }, "lightspeed_dashboard");
}

void Dashboard::dashboardLoop() {
    std::uint32_t previousTime = pros::millis();

    while (true) {
        pros::Task::delay_until(&previousTime, kDashboardLoopPeriodMs);

        // Only the driver-control period is ours to draw in: the selector
        // owns the screen while disabled, and no one draws during
        // autonomous() (it stops the selector but doesn't hand off to us).
        const bool driverControlPeriod = !pros::competition::is_disabled() && !pros::competition::is_autonomous();
        if (driverControlPeriod) {
            draw();
        }
    }
}

void Dashboard::draw() const {
    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(0, 0, kDashboardScreenWidth, kDashboardScreenHeight);

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardTitleLine, "Cherenkov -- live dashboard");

    const std::int32_t batteryMillivolts = pros::battery::get_voltage();
    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardBatteryLine, "Battery: %.2fV", batteryMillivolts / 1000.0);

    const odom::Pose pose = odometry_.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardPoseLine, "Pose: (%.1f, %.1f) in, %.1f deg", pose.xInches,
                         pose.yInches, pose.headingDegrees);

    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardConfidenceLine, "Odometry confidence: %s",
                         odom::toString(odometry_.getConfidence()));
}

}  // namespace lightspeed::telemetry
