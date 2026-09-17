# `lightspeed::vision` — AprilTag pose correction

Reads AprilTag detections from the AI Vision Sensor, solves each one into a
candidate robot field pose, gates it for trustworthiness, and (when accepted)
blends it into odometry.

> ⚠️ **This layer is not wired into the competition path.** `main.cpp` constructs
> the corrector but only ever consumes it from
> [diagnostic mode](diagnostics.md). Every constant here — calibration, mount
> offset, tag map — is a placeholder, and applying a placeholder-derived
> correction to real match odometry would silently corrupt it. See
> [checklist § 4](../BUCKET_B_CHECKLIST.md).

## Why it is shaped this way

Two deliberate design decisions, neither of them a placeholder for something
better:

**Not a full PnP solve.** Correcting drift on a close-range final approach does
not need a precise 6-DOF solve. The corner-ratio approximation used here is far
cheaper to run every cycle and accurate enough for that job.

**Discrete corrections, not continuous blending.** Vision updates are slower and
higher-latency than the 200 Hz fusion core. Rather than blending a stale reading
in every cycle, this fires a single correction only once a reading has proven
itself stable across frames.

## Types — `vision_types.hpp`

```cpp
struct CameraCalibration {
    double focalLengthPixels;
    double principalPointXPixels;
    double tagSizeInches;      // physical edge length of the printed tag
};

struct CameraMountOffset {
    double xInches;              // right of tracking center
    double yInches;              // forward of tracking center
    double headingDegreesOffset; // optical axis yaw vs. robot forward, clockwise-positive
};

struct TagWorldPose {
    std::uint8_t tagId;
    odom::Pose   pose;   // x/y = field position; heading = face normal, pointing outward
};

struct RelativeTagReading {
    double bearingDegrees;   // optical axis → tag center, clockwise-positive
    double distanceInches;   // camera → tag center
    double skewDegrees;      // tag normal's yaw vs. facing the camera squarely
};

struct VisionGatingConfig {
    std::uint8_t stableFrameThreshold;
    double       maxSkewDegrees;
    double       correctionConfidence;   // [0,1] blend weight; 1.0 = full snap
};

enum class GateReason : std::uint8_t {
    accepted, noTagDetected, unknownTagId, notStableYet, skewTooHigh
};
const char* toString(GateReason reason);
```

Conventions match `odom` exactly: heading in degrees, clockwise-positive; at
heading 0 local forward is field +y. `CameraMountOffset` uses the same
convention as `odom::PodConfig::offsetInches`.

## `tag_pose_solver.hpp`

```cpp
RelativeTagReading solveRelativePose(const hal::TagDetection& detection,
                                     const CameraCalibration& calibration);

odom::Pose backOutRobotPose(const RelativeTagReading& relative,
                            const odom::Pose& tagWorldPose,
                            const CameraMountOffset& mountOffset);
```

**Bearing** — tag-center x-offset from the principal point, via focal length.

**Distance** — physical tag size divided by the averaged length of the two side
edges (corner0–corner3 and corner1–corner2); standard pinhole size-to-distance.

**Skew** — derived from those same two edge lengths through a pinhole-projection
depth relation: a tag angled away has a near edge that projects longer than the
far edge. This is a real closed-form derivation, not a curve fit.

> ⚠️ The skew formula's **sign** depends on which physical edge the AI Vision
> Sensor's corner numbering calls "left" — unknowable without the real sensor.
> Verify it on the bench before trusting skew gating.

`backOutRobotPose()` combines a relative reading with the tag's known world pose
and the camera's fixed mount offset to produce the **robot tracking center's**
candidate field pose.

## `VisionPoseCorrector`

```cpp
struct VisionUpdate {
    bool       accepted = false;
    GateReason reason   = GateReason::noTagDetected;

    std::optional<hal::TagDetection> rawDetection;   // populated even when rejected
    RelativeTagReading relative{};                   // meaningful only with a detection
    std::uint8_t stableFrameCount = 0;               // capped at 255
    odom::Pose candidatePose{};                      // meaningful only if accepted
};

VisionPoseCorrector(hal::AiVisionSensor& sensor,
                    const CameraCalibration& calibration,
                    const CameraMountOffset& mountOffset,
                    const std::vector<TagWorldPose>& tagWorldMap,
                    const VisionGatingConfig& gating);

VisionUpdate update();
```

One cycle: read the sensor, pick the best candidate (the closest tag with a known
world pose; failing that, the closest detected tag at all, purely so diagnostics
can show *something*), solve it, track how many consecutive frames the same ID
has been seen, and gate it.

**`update()` never touches `OdometryFusion` itself.** It reports a full
diagnostic result and the caller decides whether to feed an accepted candidate
into `applyVisionCorrection()`. That separation is what lets the bench harness
observe gating decisions without acting on them.

`rawDetection` and `reason` are populated even on rejection, so you can see
*why* a reading was thrown out rather than just that nothing happened.

## `runVisionBenchTest()`

```cpp
[[noreturn]] void runVisionBenchTest(VisionPoseCorrector& corrector,
                                     odom::OdometryFusion& odometry);
```

Loops at 100 ms printing raw corner data, computed bearing/distance/skew, the
gate status and its reason, and any resulting correction — **and applies accepted
corrections to `odometry`** so their effect can be observed directly against a
physically measured tag placement.

Reachable through [diagnostic mode](diagnostics.md) (hold **Y** at boot).

## Constants — `vision_constants.hpp`

```cpp
inline constexpr CameraCalibration kAiVisionCalibration{
    .focalLengthPixels = 460.0, .principalPointXPixels = 160.0, .tagSizeInches = 6.0,
};

inline constexpr CameraMountOffset kAiVisionMountOffset{
    .xInches = 0.0, .yInches = 6.0, .headingDegreesOffset = 0.0,
};

inline const std::vector<TagWorldPose> kTagWorldMap{ /* two placeholder wall tags */ };

inline constexpr VisionGatingConfig kVisionGatingConfig{
    .stableFrameThreshold = 5,
    .maxSkewDegrees = 25.0,
    .correctionConfidence = 0.85,
};

inline constexpr std::uint32_t kVisionBenchTestLoopPeriodMs = 100;
```

> ⚠️ Calibration assumes a 320 px frame and a guessed FOV. The mount offset is
> unmeasured. The tag map is two invented wall tags. The gating thresholds are
> untested. All of it needs the bench harness against a real, physically measured
> tag placement before it is worth anything.

`correctionConfidence` at 0.85 is deliberately **high but not 1.0**: a single
noisy reading cannot fully teleport the pose, while a run of gated-in readings
converges quickly at close range.

## Bringing vision online

1. Mount the sensor. Measure `kAiVisionMountOffset` from the tracking center.
2. Get the sensor's real calibration (factory or checkerboard) into
   `kAiVisionCalibration`, along with your actual printed tag size.
3. Print one tag, place it at a **measured** field position, and put it in
   `kTagWorldMap`.
4. Boot holding **Y**, watch the bench output. Confirm distance matches a tape
   measure and bearing matches reality.
5. **Verify the skew sign** — angle the robot left and right of square and check
   that skew changes sign the way you expect.
6. Tune `stableFrameThreshold` and `maxSkewDegrees` against what the readings
   actually do.
7. Only then consider calling `applyVisionCorrection()` from the live path.
