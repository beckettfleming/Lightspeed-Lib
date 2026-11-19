/**
 * \file lightspeed/diagnostics/diagnostic_mode.hpp
 *
 * Starts the serial telemetry link in a selectable diagnostic mode gated
 * behind a boot-time controller-button hold.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/diagnostics/
 */

#pragma once

#include "lightspeed/telemetry/serial_link.hpp"

namespace lightspeed::diagnostics {

// Waits up to ~3s for the controller to connect, then samples the boot-hold
// button (see the .cpp) for ~0.5s. If it's not held, returns and does
// nothing, which is why this is safe to call unconditionally in initialize().
//
// If held, starts `serialLink` (CSV telemetry over USB) and returns, so
// driver control and autonomous still run -- you can drive the robot (e.g.
// on blocks for velocity tuning) while it streams. Power-cycle to stop.
void runDiagnosticModeIfRequested(telemetry::SerialLink& serialLink);

}  // namespace lightspeed::diagnostics
