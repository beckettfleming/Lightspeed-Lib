/**
 * \file lightspeed/vision/tag_pose_solver.hpp
 *
 * Corner-ratio approximation of a detected tag's bearing/distance/skew,
 * and combining that with the tag's known world pose + the camera's mount
 * offset to back out a candidate robot field pose. Deliberately not a full
 * PnP solve -- see the file-level comment in vision_pose_corrector.hpp for
 * why that's the intended design, not a placeholder for one.
 */

#pragma once

#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/vision/vision_types.hpp"

namespace lightspeed::vision {

// Bearing: tag-center x-offset from the principal point, via focal length.
// Distance: physical tag size / the two side edges' (corner0-corner3,
// corner1-corner2) averaged length, pinhole-camera size-to-distance.
// Skew: derived from the same two edge lengths via a pinhole-projection
// depth relation (a tag angled away has a near edge that reads longer than
// the far edge) -- see the .cpp for the exact formula. The formula itself
// is a real derivation, not an ad hoc mapping; only its sign (which edge
// the AI Vision Sensor's corner numbering calls "left") still needs
// verification against real hardware.
[[nodiscard]] RelativeTagReading solveRelativePose(const hal::TagDetection& detection, const CameraCalibration& calibration);

// Combines a relative reading with the tag's known world pose and the
// camera's fixed mount offset to compute the robot tracking-center's
// candidate field pose.
[[nodiscard]] odom::Pose backOutRobotPose(const RelativeTagReading& relative, const odom::Pose& tagWorldPose,
                                           const CameraMountOffset& mountOffset);

}  // namespace lightspeed::vision
