/**
 * \file lightspeed/diagnostics/diagnostic_mode.hpp
 *
 * Consolidates the serial telemetry link and the AprilTag vision bench test
 * -- both fully built but otherwise unreachable -- into one selectable
 * diagnostic mode gated behind a boot-time controller-button hold.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/diagnostics/
 */

#pragma once

#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/telemetry/serial_link.hpp"
#include "lightspeed/vision/vision_pose_corrector.hpp"

namespace lightspeed::diagnostics {

// If the boot-hold button (see the .cpp) is NOT held, returns immediately
// and does nothing, which is why this is safe to call unconditionally at the
// top of initialize().
//
// If held, starts `serialLink` and runs the vision bench test, which blocks
// forever. This deliberately REPLACES normal competition operation for the
// boot rather than running alongside it: initialize() never returns, so
// autonomous()/opcontrol() never run. Power-cycle to boot normally.
void runDiagnosticModeIfRequested(vision::VisionPoseCorrector& visionCorrector, odom::OdometryFusion& odometry,
                                   telemetry::SerialLink& serialLink);

}  // namespace lightspeed::diagnostics
