#include "lightspeed/diagnostics/diagnostic_mode.hpp"

#include <cstdio>

#include "pros/misc.hpp"

namespace lightspeed::diagnostics {

namespace {
constexpr pros::controller_digital_e_t kBootButton = pros::E_CONTROLLER_DIGITAL_Y;
}  // namespace

void runDiagnosticModeIfRequested(telemetry::SerialLink& serialLink) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (!master.get_digital(kBootButton)) {
        return;
    }

    std::printf(
        "\n=== DIAGNOSTIC MODE (Y held at boot) ===\n"
        "Streaming telemetry over serial. This REPLACES normal competition\n"
        "operation for this boot --\n"
        "power-cycle without Y held to boot normally.\n"
        "=========================================\n\n");

    serialLink.start();
    while (true) {
        pros::delay(1000);
    }
}

}  // namespace lightspeed::diagnostics
