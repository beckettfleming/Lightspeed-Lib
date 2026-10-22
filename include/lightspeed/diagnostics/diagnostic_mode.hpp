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

// If the boot-hold button (see the .cpp) is NOT held, returns immediately
// and does nothing, which is why this is safe to call unconditionally at the
// top of initialize().
//
// If held, starts `serialLink` and blocks forever. This deliberately
// REPLACES normal competition operation for the boot: initialize() never
// returns, so autonomous()/opcontrol() never run. Power-cycle to boot normally.
void runDiagnosticModeIfRequested(telemetry::SerialLink& serialLink);

}  // namespace lightspeed::diagnostics
