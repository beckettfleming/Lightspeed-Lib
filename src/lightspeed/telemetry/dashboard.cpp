#include "lightspeed/telemetry/dashboard.hpp"

#include "lightspeed/telemetry/dashboard_layout.hpp"
#include "lightspeed/telemetry/telemetry_constants.hpp"
#include "pros/misc.hpp"
#include "pros/screen.hpp"

namespace lightspeed::telemetry {

Dashboard::Dashboard(const odom::OdometryFusion& odometry, const control::DrivetrainVelocityController& drivetrain,
                     const std::atomic<bool>& limpModeEnabled)
    : odometry_(odometry), drivetrain_(drivetrain), limpModeEnabled_(limpModeEnabled) {}

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

        // The selector owns the screen while disabled. During enabled
        // operation, show live odometry in both autonomous and driver control.
        if (!pros::competition::is_disabled()) {
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

    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardDriveVelocityLine,
                         "Drive RPM target/measured L %.0f/%.0f R %.0f/%.0f",
                         drivetrain_.getLeftTargetVelocityRpm(), drivetrain_.getLeftMotorVelocityRpm(),
                         drivetrain_.getRightTargetVelocityRpm(), drivetrain_.getRightMotorVelocityRpm());

    drawDriveHealth();

    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardDriveModeLine, "Drive mode: %s",
                         limpModeEnabled_.load() ? "LIMP (UP = drive)" : "DRIVE (UP = limp)");
}

void Dashboard::drawDriveHealth() const {
    pros::screen::print(pros::E_TEXT_MEDIUM, kDashboardDriveHealthLine,
                         "Drive health L %s %u/%u R %s %u/%u",
                         hal::toString(drivetrain_.getLeftMotorHealth()),
                         static_cast<unsigned>(drivetrain_.getLeftConnectedMotorCount()),
                         static_cast<unsigned>(drivetrain_.getLeftMotorCount()),
                         hal::toString(drivetrain_.getRightMotorHealth()),
                         static_cast<unsigned>(drivetrain_.getRightConnectedMotorCount()),
                         static_cast<unsigned>(drivetrain_.getRightMotorCount()));
}

}  // namespace lightspeed::telemetry
