/**
 * \file lightspeed/vision/vision_bench_test.hpp
 *
 * Standalone diagnostic loop for validating AprilTag correction against a
 * physically measured tag before trusting it in a match -- prints raw
 * corner data, the computed bearing/distance/skew, gating status (pass/
 * reject and why), and the resulting correction (if any) once per loop.
 *
 * Deliberately NOT wired into main.cpp: this step is an independent
 * extension (its config -- calibration, mount offset, tag map -- is all
 * placeholder, and the AI Vision Sensor isn't necessarily even mounted on
 * the robot yet), so it doesn't touch the already-verified competition path
 * from Steps 1-8. Call runVisionBenchTest() yourself (e.g. temporarily from
 * opcontrol(), or a scratch test main) once the sensor is physically mounted
 * and vision_constants.hpp has been updated with real measurements.
 */

#pragma once

#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/vision/vision_pose_corrector.hpp"

namespace lightspeed::vision {

// Loops forever at kVisionBenchTestLoopPeriodMs (see vision_constants.hpp),
// printing one diagnostic line (or block) per cycle and applying any
// accepted correction to `odometry` so its effect can be observed directly
// against a physically measured tag placement.
[[noreturn]] void runVisionBenchTest(VisionPoseCorrector& corrector, odom::OdometryFusion& odometry);

}  // namespace lightspeed::vision
