/**
 * \file lightspeed/vision/vision_bench_test.hpp
 *
 * Standalone diagnostic loop for validating AprilTag correction against a
 * physically measured tag before trusting it in a match. Prints raw corner
 * data, computed bearing/distance/skew, gating status with the reason, and
 * the resulting correction, once per loop.
 *
<<<<<<< Updated upstream
 * Reached by holding Y at boot -- see lightspeed::diagnostics.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/diagnostics/
=======
 * Deliberately NOT wired into main.cpp: this step is an independent
 * extension (its config -- calibration, mount offset, tag map -- is all
 * placeholder, and the AI Vision Sensor isn't necessarily even mounted on
 * Cherenkov yet), so it doesn't touch the already-verified competition path
 * from Steps 1-8. Call runVisionBenchTest() yourself (e.g. temporarily from
 * opcontrol(), or a scratch test main) once the sensor is physically mounted
 * and vision_constants.hpp has been updated with real measurements.
>>>>>>> Stashed changes
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
