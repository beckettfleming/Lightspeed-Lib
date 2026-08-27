/**
 * \file lightspeed/diagnostics/diagnostic_mode.hpp
 *
 * Consolidates this project's scattered ad-hoc test entry points -- the
 * Step 8 serial telemetry link and the Step 9 AprilTag vision bench test,
 * both fully built but otherwise unwired into main.cpp -- into a single
 * selectable diagnostic mode, gated behind a boot-time controller-button
 * hold, instead of leaving them as independent one-off harnesses a
 * developer has to remember to wire in by hand.
 */

#pragma once

#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/telemetry/serial_link.hpp"
#include "lightspeed/vision/vision_pose_corrector.hpp"

namespace lightspeed::diagnostics {

// Checks a boot-time controller-button hold (see the .cpp for which
// button). If NOT held, returns immediately and does nothing -- the normal
// initialize()/opcontrol()/autonomous() competition flow proceeds
// completely unaffected, which is why this call is safe to make
// unconditionally at the top of initialize().
//
// If held, prints a banner, starts `serialLink`, and runs the vision bench
// test loop against `visionCorrector`/`odometry` (see
// lightspeed::vision::runVisionBenchTest) -- which blocks forever
// ([[noreturn]] in practice). This is a deliberate "service mode" that
// REPLACES normal competition operation for this boot rather than running
// alongside it: initialize() never returns, so autonomous()/opcontrol()
// never run this boot. Power-cycle without the button held to boot
// normally.
void runDiagnosticModeIfRequested(vision::VisionPoseCorrector& visionCorrector, odom::OdometryFusion& odometry,
                                   telemetry::SerialLink& serialLink);

}  // namespace lightspeed::diagnostics
