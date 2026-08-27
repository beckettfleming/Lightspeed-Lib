#include "lightspeed/vision/tag_pose_solver.hpp"

#include <algorithm>
#include <cmath>

namespace lightspeed::vision {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

double wrapDegrees(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped;
}

}  // namespace

RelativeTagReading solveRelativePose(const hal::TagDetection& detection, const CameraCalibration& calibration) {
    const double centerX = (detection.x0 + detection.x1 + detection.x2 + detection.x3) / 4.0;

    // Side edges: corner0-corner3 and corner1-corner2 (per the task's
    // corner pairing), NOT the top/bottom edges -- these are the ones whose
    // relative length tells us how the tag is yawed relative to the camera.
    const double edgeLeft = std::hypot(detection.x0 - detection.x3, detection.y0 - detection.y3);
    const double edgeRight = std::hypot(detection.x1 - detection.x2, detection.y1 - detection.y2);
    const double averageEdge = (edgeLeft + edgeRight) / 2.0;

    const double bearingDegrees =
        std::atan2(centerX - calibration.principalPointXPixels, calibration.focalLengthPixels) * kRadiansToDegrees;

    // Guards a degenerate (zero-size) detection rather than dividing by
    // zero -- real hardware never reports one, this is purely defensive.
    if (averageEdge <= 1e-6) {
        return RelativeTagReading{.bearingDegrees = bearingDegrees, .distanceInches = 0.0, .skewDegrees = 0.0};
    }

    const double distanceInches = (calibration.tagSizeInches * calibration.focalLengthPixels) / averageEdge;

    // Skew (relative yaw): derived from pinhole projection rather than an
    // ad hoc edge-ratio mapping. A tag yawed by angle theta about its
    // vertical axis has its near edge at camera-distance (D - w*sin(theta))
    // and its far edge at (D + w*sin(theta)), where D is the
    // already-computed center distance and w is the tag's half-width (each
    // edge's own physical length is unchanged by a yaw about the vertical
    // axis -- only its depth, and therefore its projected pixel length,
    // changes). So edgeLeft/edgeRight = depthRight/depthLeft = (D +
    // w*sin(theta)) / (D - w*sin(theta)); cross-multiplying and solving for
    // sin(theta) gives the form below. Reduces to 0 when edgeLeft ==
    // edgeRight (square-on), same as the simpler edge-ratio mapping this
    // replaced, but is now a proper closed-form derivation rather than an
    // arbitrary acos(shorter/longer).
    const double halfTagWidth = calibration.tagSizeInches / 2.0;
    const double sinSkewDenominator = halfTagWidth * (edgeLeft + edgeRight);
    const double sinSkew = sinSkewDenominator > 1e-6
                                ? std::clamp(distanceInches * (edgeLeft - edgeRight) / sinSkewDenominator, -1.0, 1.0)
                                : 0.0;
    const double skewDegrees = std::asin(sinSkew) * kRadiansToDegrees;

    // The formula's sign (edgeLeft nearer -> positive) is now principled,
    // not asserted -- but WHICH edge is physically "left" still depends on
    // the AI Vision Sensor's own corner-numbering convention, which is a
    // hardware fact this project doesn't have without a real sensor. Same
    // caveat as odom::PodConfig::offsetInches: verify on the bench harness
    // (vision_bench_test.hpp) against a tag rotated a known way, and negate
    // this whole expression if the sign comes out backwards.
    return RelativeTagReading{
        .bearingDegrees = bearingDegrees,
        .distanceInches = distanceInches,
        .skewDegrees = skewDegrees,
    };
}

odom::Pose backOutRobotPose(const RelativeTagReading& relative, const odom::Pose& tagWorldPose,
                             const CameraMountOffset& mountOffset) {
    // At zero skew the camera's optical axis points directly opposite the
    // tag's outward face normal; skew rotates it away from that square-on
    // line, and bearing further rotates the specific camera->tag ray away
    // from the camera's own optical axis.
    const double cameraOpticalAxisHeadingDegrees = wrapDegrees(tagWorldPose.headingDegrees + 180.0 + relative.skewDegrees);
    const double cameraToTagHeadingDegrees = wrapDegrees(cameraOpticalAxisHeadingDegrees + relative.bearingDegrees);
    const double cameraToTagHeadingRadians = cameraToTagHeadingDegrees * kDegreesToRadians;

    // Camera position = tag position, walked back `distance` along the
    // reverse of the camera->tag ray (heading-to-vector: (sin, cos), same
    // convention as lightspeed::odom / lightspeed::motion).
    const double cameraFieldX = tagWorldPose.xInches - relative.distanceInches * std::sin(cameraToTagHeadingRadians);
    const double cameraFieldY = tagWorldPose.yInches - relative.distanceInches * std::cos(cameraToTagHeadingRadians);

    const double robotHeadingDegrees = wrapDegrees(cameraOpticalAxisHeadingDegrees - mountOffset.headingDegreesOffset);
    const double robotHeadingRadians = robotHeadingDegrees * kDegreesToRadians;

    // Invert the local->field mount-offset rotation (see pose_math.hpp's
    // toLocalFrame doc comment for the same rotation used in reverse) to
    // get from the camera's field position back to the tracking center's.
    const double mountFieldDX = mountOffset.yInches * std::sin(robotHeadingRadians) + mountOffset.xInches * std::cos(robotHeadingRadians);
    const double mountFieldDY = mountOffset.yInches * std::cos(robotHeadingRadians) - mountOffset.xInches * std::sin(robotHeadingRadians);

    return odom::Pose{
        .xInches = cameraFieldX - mountFieldDX,
        .yInches = cameraFieldY - mountFieldDY,
        .headingDegrees = robotHeadingDegrees,
    };
}

}  // namespace lightspeed::vision
