#include "lightspeed/diagnostics/diagnostic_mode.hpp"

#include <cstdio>

#include "lightspeed/vision/vision_bench_test.hpp"
#include "pros/misc.hpp"

namespace lightspeed::diagnostics {

namespace {
// Chosen to not collide with any direct button binding already in
// opcontrol() (R1/R2 -> exampleArm presets, L1 -> the demo button macro).
constexpr pros::controller_digital_e_t kBootButton = pros::E_CONTROLLER_DIGITAL_Y;
}  // namespace

void runDiagnosticModeIfRequested(vision::VisionPoseCorrector& visionCorrector, odom::OdometryFusion& odometry,
                                   telemetry::SerialLink& serialLink) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (!master.get_digital(kBootButton)) {
        return;
    }

    std::printf(
        "\n=== DIAGNOSTIC MODE (Y held at boot) ===\n"
        "Streaming telemetry over serial + running the AprilTag vision bench\n"
        "test. This REPLACES normal competition operation for this boot --\n"
        "power-cycle without Y held to boot normally.\n"
        "=========================================\n\n");

    serialLink.start();
    vision::runVisionBenchTest(visionCorrector, odometry);  // never returns
}

}  // namespace lightspeed::diagnostics
