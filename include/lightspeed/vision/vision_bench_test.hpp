/**
 * \file lightspeed/vision/vision_bench_test.hpp
 *
 * Standalone diagnostic loop for validating AprilTag correction against a
 * physically measured tag before trusting it in a match. Prints raw corner
 * data, computed bearing/distance/skew, gating status with the reason, and
 * the resulting correction, once per loop.
 *
 * Reached by holding Y at boot -- see lightspeed::diagnostics.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/diagnostics/
 */

#pragma once

#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/vision/vision_pose_corrector.hpp"

namespace lightspeed::vision {

// Loops forever, applying any accepted correction to `odometry` so its
// effect can be observed against a physically measured tag placement.
[[noreturn]] void runVisionBenchTest(VisionPoseCorrector& corrector, odom::OdometryFusion& odometry);

}  // namespace lightspeed::vision
