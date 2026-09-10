---
title: Vision
parent: Layers
nav_order: 9
permalink: /layers/vision/
---

**Namespace:** `lightspeed::vision` · **Headers:** `include/lightspeed/vision/`

Uses the VEX AI Vision Sensor to spot AprilTags with known field positions and correct odometry
drift from them. Intended for close-range final approach, where a couple of inches of
accumulated drift matters most.

> ⚠️ **This layer is deliberately NOT wired into the competition path.** Its calibration, mount
> offset, and tag map are all placeholders, and applying a placeholder-derived correction to
> real match odometry would silently corrupt it. It is reachable only through
> [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/). See [Wiring it up](#wiring-it-into-competition) below.

---

## The chain

```
  AI Vision Sensor
        |  raw pixel corners of each detected tag
        v
  hal::AiVisionSensor::getDetectedTags()
        |
        v
  solveRelativePose()          corners -> bearing, distance, skew
        |
        v
  backOutRobotPose()           + tag world pose + camera mount offset
        |                        -> candidate robot field pose
        v
  VisionPoseCorrector::update()
        |  gate it: known tag? stable enough? skew acceptable?
        v
  VisionUpdate { accepted, reason, candidatePose, ... }
        |
        |  the CALLER decides whether to apply it
        v
  OdometryFusion::applyVisionCorrection(pose, weight)   <- weighted blend
```

**`update()` never touches odometry itself.** It reports a full diagnostic result and lets the
caller decide. That separation is what makes the bench harness possible without a special mode
inside the corrector.

---

## Why not a full PnP solve

A proper 6-DOF Perspective-n-Point solve is the textbook answer, and this project deliberately
doesn't do one:

- **Correcting drift on a close-range final approach doesn't need 6 DOF.** Bearing, distance,
  and yaw are enough.
- **The corner-ratio approximation is far cheaper** to run every cycle on a V5 brain.
- **Corrections are gated, not continuous.** Vision is slower and higher-latency than the 200 Hz
  fusion core, so this fires a discrete correction once a reading is actually trustworthy rather
  than blending noise in every cycle.

This is a design decision, not a placeholder for a "real" implementation later.

---

## Types

```cpp
struct CameraCalibration {
    double focalLengthPixels;
    double principalPointXPixels;
    double tagSizeInches;          // physical edge length of the printed tag
};

struct CameraMountOffset {
    double xInches;                // right of the robot's tracking center
    double yInches;                // forward of the tracking center
    double headingDegreesOffset;   // camera yaw vs. robot forward, clockwise-positive
};

struct TagWorldPose {
    std::uint8_t tagId;
    odom::Pose pose;               // field position; heading = the face normal's outward direction
};

struct RelativeTagReading {
    double bearingDegrees;    // optical axis -> tag center, clockwise-positive
    double distanceInches;    // camera -> tag center
    double skewDegrees;       // tag normal's yaw vs. facing the camera squarely
};

struct VisionGatingConfig {
    std::uint8_t stableFrameThreshold;
    double maxSkewDegrees;
    double correctionConfidence;   // [0,1] blend weight; 1.0 == a full snap
};

enum class GateReason : std::uint8_t {
    accepted, noTagDetected, unknownTagId, notStableYet, skewTooHigh
};
```

All of it uses the same convention as odometry: heading in degrees, clockwise-positive; at
heading 0, forward is field +y. See [Coordinate System]({{ site.baseurl }}/reference/coordinates/).

---

## `solveRelativePose()`

Turns one tag's four pixel corners into bearing, distance, and skew.

| Value | How it's derived |
|---|---|
| **Bearing** | Tag-center x-offset from the principal point, via focal length |
| **Distance** | Physical tag size divided by the averaged length of the two side edges — standard pinhole size-to-distance |
| **Skew** | From the *ratio* of those two edge lengths, via a pinhole-projection depth relation: a tag angled away has a near edge that reads longer than its far edge |

The two side edges are corner0→corner3 and corner1→corner2.

### The skew formula's sign

The formula is a **real closed-form derivation** from the pinhole projection using the
already-computed distance and known tag width — it replaced an earlier ad hoc
`acos(shorterEdge / longerEdge)` curve fit. It correctly reduces to 0 when the tag is square-on.

**Its sign, however, still needs on-hardware verification.** Which physical edge the AI Vision
Sensor's corner numbering calls "left" isn't knowable without a real sensor. If corrections push
the pose consistently the wrong way laterally, flip the sign — see the comment in
[`tag_pose_solver.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/vision/tag_pose_solver.cpp).

---

## `backOutRobotPose()`

Combines a relative reading with the tag's known world pose and the camera's fixed mount offset
to produce the **robot tracking center's** candidate field pose.

The mount offset matters more than people expect. A camera 6 inches forward of the tracking
center produces a 6-inch error in every correction if `kAiVisionMountOffset` is wrong. Measure
it.

---

## `VisionPoseCorrector`

```cpp
vision::VisionPoseCorrector corrector(
    aiVisionSensor,
    vision::kAiVisionCalibration,
    vision::kAiVisionMountOffset,
    vision::kTagWorldMap,
    vision::kVisionGatingConfig);

vision::VisionUpdate update = corrector.update();   // one full cycle

if (update.accepted) {
    odometry.applyVisionCorrection(update.candidatePose,
                                   vision::kVisionGatingConfig.correctionConfidence);
}
```

### What `update()` returns

```cpp
struct VisionUpdate {
    bool accepted;
    GateReason reason;
    std::optional<hal::TagDetection> rawDetection;  // populated even when REJECTED
    RelativeTagReading relative;                     // valid if rawDetection has a value
    std::uint8_t stableFrameCount;                   // consecutive frames of the same ID
    odom::Pose candidatePose;                        // valid only if accepted
};
```

The raw detection and relative reading are populated **even on rejection**, so the bench harness
can show exactly what was seen and why it was thrown out.

### Tag selection

The closest tag whose ID appears in the world map. If none of the detected tags are known, it
falls back to the closest detected tag **purely for diagnostics** — that result is rejected with
`unknownTagId`, but you can still see it.

### The three gates

A reading must pass all of them:

| Gate | Rejection reason | Why |
|---|---|---|
| The tag ID is in the world map | `unknownTagId` | An unmapped tag has no known field position |
| Seen for `stableFrameThreshold` consecutive frames | `notStableYet` | Filters flicker and one-frame false positives |
| `|skew| <= maxSkewDegrees` | `skewTooHigh` | A steeply angled tag gives a poor distance estimate |

And if nothing was detected at all: `noTagDetected`.

### Corrections blend, they don't snap

`correctionConfidence` is currently **0.85** — high, but deliberately not 1.0, so a single noisy
reading can't fully teleport the pose. A run of gated-in readings converges on the true pose
quickly at close range, which is exactly the situation this is built for.

`applyVisionCorrection()` blends heading along the **shortest signed path**, so a correction
across the 0/360 boundary doesn't spin the pose the long way around, and it does **not** reset
odometry source baselines — it's a nudge to a running estimate, not a re-initialization. See
[Odometry Layer]({{ site.baseurl }}/layers/odometry/).

---

## The bench harness

```cpp
[[noreturn]] void runVisionBenchTest(VisionPoseCorrector& corrector,
                                     odom::OdometryFusion& odometry);
```

Loops forever at 10 Hz, printing one diagnostic block per cycle: raw corner data, computed
bearing/distance/skew, gate status with the reason, and the resulting correction if any. It
**applies** accepted corrections to `odometry` so you can watch their effect directly against a
physically measured tag placement.

Reached by holding **Y at boot** — see [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/).

This is how you calibrate the whole layer: place a tag at a measured spot, put the robot at a
measured spot, and check whether the computed pose matches reality.

---

## Configuration

[`include/lightspeed/vision/vision_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/vision/vision_constants.hpp)

```cpp
inline constexpr CameraCalibration kAiVisionCalibration{
    .focalLengthPixels = 460.0,        // TODO: real calibration
    .principalPointXPixels = 160.0,    // assumes a 320px-wide frame
    .tagSizeInches = 6.0,              // TODO: your actual printed tag size
};

inline constexpr CameraMountOffset kAiVisionMountOffset{
    .xInches = 0.0, .yInches = 6.0, .headingDegreesOffset = 0.0,   // TODO: measure
};

inline const std::vector<TagWorldPose> kTagWorldMap{   // TODO: real season layout
    TagWorldPose{.tagId = 1, .pose = {  0.0, 72.0,  90.0}},
    TagWorldPose{.tagId = 2, .pose = {144.0, 72.0, 270.0}},
};

inline constexpr VisionGatingConfig kVisionGatingConfig{
    .stableFrameThreshold = 5,      // TODO: bench-tune
    .maxSkewDegrees = 25.0,         // TODO: bench-tune
    .correctionConfidence = 0.85,
};

inline constexpr std::uint32_t kVisionBenchTestLoopPeriodMs = 100;
```

> 🚧 **Every value above is a placeholder.** None of it should be trusted until measured against
> real hardware and the real field.

---

## Bringing this online

In order:

1. **Mount the sensor** and set its port in `hal/config.hpp`.
2. **Confirm the tag family** (`kAiVisionSensor.tagFamily`, currently `tag_16H5`) against what
   the season's field elements actually use.
3. **Calibrate the camera** — get the real focal length and principal point from its factory
   calibration or a checkerboard pass.
4. **Measure the mount offset** from the robot's tracking center.
5. **Print tags** and set `tagSizeInches` to the real printed edge length.
6. **Enter the real tag map** once the season's field layout is published.
7. **Run the bench harness** (hold Y at boot) with a tag at a measured position, and verify the
   computed pose matches reality. Fix the skew sign here if corrections push the wrong way.
8. **Tune the gates** — `stableFrameThreshold` and `maxSkewDegrees` — against real readings.

### Wiring it into competition

Only after all eight steps. In `opcontrol()` or a motion primitive's loop:

```cpp
const vision::VisionUpdate update = gVisionCorrector->update();
if (update.accepted) {
    gOdometry->applyVisionCorrection(update.candidatePose,
                                     vision::kVisionGatingConfig.correctionConfidence);
}
```

`applyVisionCorrection()` is **purely additive** to `OdometryFusion` — it doesn't restructure the
normal arc-based update loop at all, so turning vision on doesn't change how odometry works when
no tag is visible.

---

**Next:** [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/) — how to reach the bench harness.
