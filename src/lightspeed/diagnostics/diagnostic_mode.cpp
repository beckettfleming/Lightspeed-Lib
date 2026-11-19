#include "lightspeed/diagnostics/diagnostic_mode.hpp"

#include <cstdint>
#include <cstdio>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::diagnostics {

namespace {
constexpr pros::controller_digital_e_t kBootButton = pros::E_CONTROLLER_DIGITAL_Y;

// A wireless controller can take a moment to (re)link after the program
// starts; until it does, every button reads as released.
constexpr std::uint32_t kControllerConnectWaitMs = 3000;

// How long Y is sampled for. Y held at ANY point in this window counts, so a
// single unlucky read can't miss it.
constexpr std::uint32_t kBootButtonWindowMs = 500;
constexpr std::uint32_t kPollPeriodMs = 10;
}  // namespace

void runDiagnosticModeIfRequested(telemetry::SerialLink& serialLink) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    const std::uint32_t connectStart = pros::millis();
    while (!master.is_connected() && pros::millis() - connectStart < kControllerConnectWaitMs) {
        pros::delay(kPollPeriodMs);
    }
    if (!master.is_connected()) {
        std::printf("[diagnostics] controller not connected -- skipping diagnostic-mode check\n");
        return;
    }

    bool requested = false;
    const std::uint32_t windowStart = pros::millis();
    while (pros::millis() - windowStart < kBootButtonWindowMs) {
        if (master.get_digital(kBootButton)) {
            requested = true;
            break;
        }
        pros::delay(kPollPeriodMs);
    }
    if (!requested) {
        std::printf("[diagnostics] Y not held at startup -- normal boot, no serial telemetry\n");
        return;
    }

    std::printf(
        "\n=== DIAGNOSTIC MODE (Y held at startup) ===\n"
        "Streaming telemetry over serial as CSV. Driver control and\n"
        "autonomous still run normally, so you can drive while streaming.\n"
        "Power-cycle without Y held to stop streaming.\n"
        "============================================\n\n");
    master.rumble(".");

    serialLink.start();
}

}  // namespace lightspeed::diagnostics
