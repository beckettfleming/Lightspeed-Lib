#include "lightspeed/vision/vision_bench_test.hpp"

#include <cstdio>

#include "lightspeed/vision/vision_constants.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::vision {

namespace {

void printCycle(const VisionUpdate& result) {
    if (!result.rawDetection.has_value()) {
        std::printf("[vision] no tag detected\n");
        return;
    }

    const hal::TagDetection& d = *result.rawDetection;
    std::printf("[vision] tag id=%u corners=(%.0f,%.0f)(%.0f,%.0f)(%.0f,%.0f)(%.0f,%.0f)\n", d.id, d.x0, d.y0, d.x1,
                d.y1, d.x2, d.y2, d.x3, d.y3);
    std::printf("[vision]   bearing=%.2fdeg distance=%.2fin skew=%.2fdeg stableFrames=%u\n", result.relative.bearingDegrees,
                result.relative.distanceInches, result.relative.skewDegrees, result.stableFrameCount);
    std::printf("[vision]   gate=%s\n", toString(result.reason));

    if (result.accepted) {
        std::printf("[vision]   correction applied: candidate pose=(%.2f, %.2f, %.1fdeg)\n", result.candidatePose.xInches,
                    result.candidatePose.yInches, result.candidatePose.headingDegrees);
    } else {
        std::printf("[vision]   no correction applied\n");
    }
}

}  // namespace

void runVisionBenchTest(VisionPoseCorrector& corrector, odom::OdometryFusion& odometry) {
    std::uint32_t previousTime = pros::millis();

    while (true) {
        pros::Task::delay_until(&previousTime, kVisionBenchTestLoopPeriodMs);

        const VisionUpdate result = corrector.update();
        printCycle(result);

        if (result.accepted) {
            const odom::Pose beforePose = odometry.getPose();
            odometry.applyVisionCorrection(result.candidatePose, kVisionGatingConfig.correctionConfidence);
            const odom::Pose afterPose = odometry.getPose();
            std::printf("[vision]   odometry pose (%.2f, %.2f, %.1fdeg) -> (%.2f, %.2f, %.1fdeg)\n", beforePose.xInches,
                        beforePose.yInches, beforePose.headingDegrees, afterPose.xInches, afterPose.yInches,
                        afterPose.headingDegrees);
        }
    }
}

}  // namespace lightspeed::vision
