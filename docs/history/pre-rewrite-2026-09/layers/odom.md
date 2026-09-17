# `lightspeed::odom` — odometry

Fuses motor encoders (IMEs), one or two IMUs, and 0–4 tracking-wheel pods into a
single field-frame pose, on its own **~200 Hz task**.

The central design constraint: the pod count and role mix are **configuration,
not code**. Today's robot might run zero pods; the next one might run three in a
different arrangement. The resolver handles any topology, degrades through a
confidence hierarchy as sensors drop out, and reports how much it currently
trusts itself.

## Types — `types.hpp`

```cpp
enum class PodRole : std::uint8_t { forward, strafe };
enum class DrivetrainKinematics : std::uint8_t { tank, holonomic };
enum class ConfidenceTier : std::uint8_t { fullPod = 0, partial = 1, imeOnly = 2 };

const char* toString(ConfidenceTier tier);
```

`ConfidenceTier`'s declaration order **is** its rank, best to worst.

```cpp
struct Pose {
    double xInches = 0.0;
    double yInches = 0.0;
    double headingDegrees = 0.0;   // clockwise-positive, wrapped [0, 360)
};

struct Velocity {
    double xInchesPerSecond = 0.0;
    double yInchesPerSecond = 0.0;
    double headingDegreesPerSecond = 0.0;
};
```

```cpp
struct PodConfig {
    const char* name;
    PodRole     role;
    double      offsetInches;    // lever arm — see below
    double      ticksToInches;   // wheel circumference / 36000 for a Rotation sensor
};

struct TopologyConfig {
    DrivetrainKinematics kinematics;
    std::vector<PodConfig> pods;
};
```

### `offsetInches`, precisely

Signed offset from the tracking center, measured along the axis
**perpendicular to that pod's own rolling direction**:

* A **forward** pod's offset is its left/right position — positive = right.
* A **strafe** pod's offset is its forward/back position — positive = forward.

This is the lever arm used for rotation correction. **Verify the sign
empirically**: spin the robot in place and watch x/y. Nonzero drift during a
pure turn means a sign is flipped.

> ⚠️ **The source currently contradicts itself here.** `PodConfig`'s doc comment
> in `types.hpp` says positive = right, but `kCherenkovTopology` in
> `odometry_constants.hpp` labels `leftForwardPod`'s `+6.0` as "left of tracking
> center." One of the two is wrong. Resolve it on the bench with the rotation
> test below before trusting either, and fix whichever comment is incorrect.

`PodConfig` deliberately holds no port number and no sensor reference — those
live in `hal::config` and are paired with a `PodConfig` only where a
`TrackingWheelSource` is constructed, so the geometry description stays reusable.

## Sources

Each source converts raw HAL units to inches or degrees and reports **deltas
since the last read**, with an internal baseline you can reset.

### `IMESource`

Forward distance from a drive motor group's built-in encoders. This is the
zero-pod fallback. Construct one per drive side so the fusion core can average
them.

```cpp
struct IMEConfig {
    double wheelDiameterInches;
    double gearRatio;   // wheel revs per motor-output-shaft rev
};

IMESource(hal::MotorGroup& motors, const IMEConfig& config);

double readDeltaInches();   // 0.0 if unhealthy or first read after reset
bool   isHealthy() const;
void   resetBaseline();
```

A **stalled or overheating motor still reports a valid encoder position** — only
a fully disconnected group invalidates this source.

### `IMUSource`

Heading from one or two `hal::Imu`s, averaged when both are present and ready.

```cpp
explicit IMUSource(hal::Imu& primary, hal::Imu* secondary = nullptr);

double readHeadingDeltaDegrees();
bool   isHealthy() const;         // true if at least one IMU is ready
void   resetBaseline();
double getHeadingDegrees() const; // absolute [0,360), for display only
```

Pass `nullptr` for a single-IMU robot. Averaging assumes both IMUs were
calibrated together so their rotation references start aligned. Delta math uses
continuous rotation internally, never the wrapped heading.

### `TrackingWheelSource`

One Rotation-sensor pod.

```cpp
TrackingWheelSource(hal::RotationSensor& sensor, const PodConfig& config);

double readDeltaInches();
bool   isHealthy() const;
void   resetBaseline();
const PodConfig& getConfig() const;
```

## `OdometryFusion`

```cpp
OdometryFusion(DrivetrainKinematics kinematics,
               IMESource& leftIme, IMESource& rightIme, IMUSource& imu,
               const std::vector<TrackingWheelSource*>& pods);

Pose           getPose()       const;
Velocity       getVelocity()   const;
ConfidenceTier getConfidence() const;

void setPose(const Pose& pose);
void applyVisionCorrection(const Pose& visionPose, double confidenceWeight);
```

Non-copyable and non-movable — it owns a task referencing `this`. Pod pointers
must outlive the object. `kMaxPodsPerRole = 4` bounds each role's array (the
worst case is every pod sharing one role), not the whole-robot pod count.

### The per-cycle algorithm

Every 5 ms:

1. **Read everything.** All sources are read unconditionally, even ones that end
   up unused, so their internal baselines stay current.
2. **Resolve heading first** — the pod correction below needs Δθ:
   * IMU if healthy (always primary), else
   * the differential between the widest-spaced healthy forward pod pair, else
   * the same for strafe pods, else
   * heading holds unchanged.
3. **Resolve each axis.** For every healthy pod, subtract the arc contributed
   purely by rotation about the tracking center
   (`delta − offsetInches × Δθ`), leaving translation. Then average across two
   healthy pods, or take the median across three or more. If an axis has **zero**
   healthy pods, fall back to drivetrain kinematics — IME average for forward,
   zero for strafe under tank.
4. **Chord correction.** Convert the arc swept this cycle into the straight-line
   chord that belongs in x/y: `2·sin(Δθ/2) / Δθ`, converging to 1 as Δθ → 0.
5. **Rotate into the field frame** using the **midpoint heading** — the average
   of heading before and after this cycle, not the stale value.
6. **Integrate**, compute velocity, publish pose/velocity/confidence and record
   telemetry.

### Confidence tiering

Per axis: no configured pods or no healthy pods → `imeOnly`; all configured pods
healthy → `fullPod`; otherwise → `partial`. The reported tier is the **worse** of
the two axes.

`getConfidence()` is not decoration — a routine that cares about accuracy should
check it, and it is one of the more useful things on the dashboard when
something goes wrong mid-match.

### `setPose()` vs. `applyVisionCorrection()`

| | `setPose()` | `applyVisionCorrection()` |
| --- | --- | --- |
| Purpose | (Re-)initialization | Correction to a running estimate |
| Source baselines | **Reset**, atomically with the pose | **Preserved** |
| Effect | Hard set | Weighted blend toward `visionPose` |
| Typical caller | Selector GUI on start-location tap | Vision corrector |

`confidenceWeight` is in `[0, 1]`: 1.0 is a full snap, less nudges the pose part
of the way. The blend is deliberate — a single noisy reading must not be able to
teleport the pose. Neither call disturbs the normal arc-based update loop.

## Constants — `odometry_constants.hpp`

```cpp
inline constexpr double kTrackingWheelDiameterInches = 2.0;
inline constexpr double kTrackingWheelTicksToInches =
    (kPi * kTrackingWheelDiameterInches) / 36000.0;   // centidegrees → inches

inline const TopologyConfig kCherenkovTopology{ /* 2 forward + 1 strafe */ };

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 4.0,
    .gearRatio = 343.0 / 600.0,
};
```

> ⚠️ The wheel diameter, all three pod offsets, and the gear ratio are
> placeholders. The topology itself is a guess at a common 3-pod tank layout.
> See [checklist § 2](../BUCKET_B_CHECKLIST.md) and
> [Getting started § 3](../getting-started.md#3-odometry-topology--includelightspeedodomodometry_constantshpp).

**Pod order matters.** `main.cpp` pairs `kCherenkovTopology.pods[i]` with the
`i`-th `hal::RotationSensor` by index. If you reorder the vector, reorder the
construction too.

## Telemetry published

| Channel | Type |
| --- | --- |
| `odom.pose.x` / `.y` / `.heading` | number |
| `odom.velocity.x` / `.y` / `.headingRate` | number |
| `odom.confidence` | text |
| `odom.forwardPods.healthyCount` / `odom.strafePods.healthyCount` | integer |
| `hal.leftIme.healthy` / `hal.rightIme.healthy` / `hal.imu.healthy` | boolean |
