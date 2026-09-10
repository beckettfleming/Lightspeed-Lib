/**
 * \file lightspeed/vision/tag_pose_solver.hpp
 *
 * Corner-ratio approximation of a detected tag's bearing/distance/skew, and
 * combining that with the tag's known world pose + the camera's mount offset
 * to back out a candidate robot field pose. Deliberately not a full PnP
 * solve -- see vision_pose_corrector.hpp for why that's intended.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/vision/
 */

#pragma once

#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/vision/vision_types.hpp"

namespace lightspeed::vision {

// Bearing: tag-center x-offset from the principal point, via focal length.
// Distance: tag size / the averaged length of the two side edges
// (corner0-corner3, corner1-corner2).
// Skew: the ratio of those edges, via a pinhole-projection depth relation.
//
// The skew formula is a real derivation, not an ad hoc mapping -- but its
// SIGN (which edge the sensor's corner numbering calls "left") still needs
// verification against real hardware. See the .cpp.
[[nodiscard]] RelativeTagReading solveRelativePose(const hal::TagDetection& detection, const CameraCalibration& calibration);

// Combines a relative reading with the tag's known world pose and the
// camera's fixed mount offset to compute the robot tracking-center's
// candidate field pose.
[[nodiscard]] odom::Pose backOutRobotPose(const RelativeTagReading& relative, const odom::Pose& tagWorldPose,
                                           const CameraMountOffset& mountOffset);

}  // namespace lightspeed::vision
