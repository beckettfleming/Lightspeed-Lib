---
title: Odometry
parent: Layers
nav_order: 3
permalink: /layers/odometry/
---

**Namespace:** `lightspeed::odom` · **Headers:** `include/lightspeed/odom/`

Odometry answers "where is the robot?" — continuously, in field coordinates, at 200 Hz. Every
autonomous motion primitive reads from here.

---

## What this robot actually uses

> **The current robot has no tracking-wheel pods.** Odometry runs on **drive-motor encoders
> (IMEs) plus dual IMU**. `kOdometryTopology.pods` is intentionally empty.

The full 0–4-pod fusion machinery is built and working — it's just unused right now. If pods
get added later, you populate `kOdometryTopology.pods` and construct `TrackingWheelSource`
objects; nothing else changes.

```
   +----------------+   +----------------+   +------------------+
   | Left drive IME |   | Right drive IME|   |  Primary IMU     |
   +--------+-------+   +--------+-------+   |  Secondary IMU   |
            |                    |           +---------+--------+
            v                    v                     v
      IMESource            IMESource               IMUSource
      (inches)             (inches)                (degrees, averaged)
            |                    |                     |
            +----------+---------+---------------------+
                       v
              +--------------------+       (optional, unused today)
              |  OdometryFusion    | <---- TrackingWheelSource x 0-4
              |  200 Hz task       |
              +---------+----------+
                        |
                        v
             Pose (x, y, heading) + Velocity + ConfidenceTier
```

---

## Coordinate convention

This is used identically by odometry, motion, and vision. Getting it wrong is the single most
common source of "the robot drove the wrong way".

- **Heading:** degrees, **clockwise-positive**, wrapped to `[0, 360)`
- **At heading 0:** robot forward → field **+y**; robot right → field **+x**
- **Distance:** inches

Full explanation with diagrams on **[Coordinate System]({{ site.baseurl }}/reference/coordinates/)**.

---

## The data types — `types.hpp`

```cpp
struct Pose {
    double xInches = 0.0;
    double yInches = 0.0;
    double headingDegrees = 0.0;   // clockwise-positive, [0, 360)
};

struct Velocity {
    double xInchesPerSecond = 0.0;
    double yInchesPerSecond = 0.0;
    double headingDegreesPerSecond = 0.0;
};

enum class ConfidenceTier : std::uint8_t { fullPod = 0, partial = 1, imeOnly = 2 };
enum class PodRole : std::uint8_t { forward, strafe };
enum class DrivetrainKinematics : std::uint8_t { tank, holonomic };
```

`ConfidenceTier`'s declaration order **is** its rank, best to worst. That's what lets
`worseOf()` be a plain `std::max`.

| Tier | Meaning |
|---|---|
| `fullPod` | Every configured pod on this axis is healthy |
| `partial` | Some but not all configured pods are healthy |
| `imeOnly` | No healthy pods — falling back to drive encoders |

With zero pods configured (today's robot), confidence is **always `imeOnly`**. That's expected,
not a fault.

---

## The sources

Each source converts one sensor into a **delta since the last read**. All of them return `0.0`
on their first read after construction or `resetBaseline()`, so a stale baseline can never
produce a phantom jump.

### `IMESource` — forward distance from drive encoders

```cpp
struct IMEConfig {
    double wheelDiameterInches;
    double gearRatio;   // wheel revolutions per motor-output-shaft revolution
};

odom::IMESource leftIme(leftMotorGroup, odom::kDriveImeConfig);
double deltaInches = leftIme.readDeltaInches();
```

One per drive side, so the fusion core can average them.

A **stalled or over-temperature** motor still reports a valid encoder position, so those don't
invalidate this source. Only an actually **disconnected** motor group does.

### `IMUSource` — heading from one or two IMUs

```cpp
odom::IMUSource imu(primaryImu, &secondaryImu);   // second may be nullptr

double deltaDegrees = imu.readHeadingDeltaDegrees();
bool ok = imu.isHealthy();                 // true if AT LEAST ONE IMU is ready
double display = imu.getHeadingDegrees();  // [0,360), display only
```

When both IMUs are present and ready, their **continuous** rotation readings are averaged. This
assumes both were calibrated together so their references start aligned — which is why
`initialize()` calibrates both, blocking, back to back.

Delta math uses continuous rotation, never the wrapped `[0,360)` value, so a turn through 0°
doesn't produce a 359° jump.

### `TrackingWheelSource` — distance from a rotation-sensor pod

Unused on this robot, but fully implemented.

```cpp
struct PodConfig {
    const char* name;
    PodRole role;            // forward or strafe
    double offsetInches;     // signed lever arm from the tracking center
    double ticksToInches;    // centidegrees -> inches (circumference / 36000)
};
```

**`offsetInches` is measured along the axis *perpendicular* to the pod's own rolling
direction.** A forward pod's offset is its left/right position; a strafe pod's offset is its
forward/back position. Positive = right (forward pod) / forward (strafe pod).

If a pure in-place turn shows nonzero x/y drift, the sign is wrong — flip it.

---

## `OdometryFusion`

Fuses everything into a single field-frame pose on its own **200 Hz task**.

### Interface

```cpp
odom::OdometryFusion odometry(
    odom::kOdometryTopology.kinematics,
    leftIme, rightIme, imuSource,
    std::vector<odom::TrackingWheelSource*>{});   // empty: no pods

odom::Pose pose = odometry.getPose();
odom::Velocity vel = odometry.getVelocity();
odom::ConfidenceTier conf = odometry.getConfidence();

odometry.setPose({.xInches = 12.0, .yInches = 12.0, .headingDegrees = 0.0});
odometry.applyVisionCorrection(visionPose, 0.85);
```

Non-copyable and non-movable — it owns a background task holding a `this` pointer. The task is
also **deferred-start**: it's constructed at the *end* of the constructor body, after pod
partitioning finishes, so it can never observe half-built state.

### One fusion cycle, step by step

```
  every 5 ms:
  ---------------------------------------------------------------
  1. READ EVERYTHING, unconditionally
     Even sources that end up unused this cycle get read, so their
     internal baselines stay current and can't jump later.

  2. RESOLVE HEADING (must come first — the lever-arm math needs it)
     IMU healthy?               -> use the IMU delta
     else pod pair available?   -> derive it differentially from two pods
     else                       -> heading holds at its last value

  3. RESOLVE EACH AXIS (forward, then strafe)
     For each healthy pod:
        corrected = rawDelta - (podOffset * deltaThetaRadians)
        ...i.e. subtract the arc this pod swept purely from rotating.
     Then combine:
        1 pod   -> use it
        2 pods  -> average
        3+ pods -> MEDIAN (one snagged pod gets outvoted, not averaged in)
     No healthy pods on this axis -> use the kinematics fallback:
        forward: average the healthy drive IMEs
        strafe:  exactly 0 for tank (a tank robot cannot strafe)

  4. ARC-TO-CHORD CORRECTION
     The robot swept an arc; x/y needs the straight-line chord:
        chordFactor = 2 * sin(dTheta / 2) / dTheta      (-> 1 as dTheta -> 0)

  5. ROTATE INTO THE FIELD FRAME using the MIDPOINT heading
     (average of before and after this cycle, not the stale old value):
        dx = forward * sin(midHeading) + strafe * cos(midHeading)
        dy = forward * cos(midHeading) - strafe * sin(midHeading)

  6. INTEGRATE, compute velocity, assign a confidence tier, publish
```

Two details worth internalizing:

- **Midpoint heading, not stale heading.** Using the heading from the *start* of the cycle
  systematically biases the pose during any turn. Averaging start and end removes most of that.
- **Median for 3+ pods.** If one pod snags on a field element, an average drags the whole
  estimate off. A median outvotes it.

### `setPose()` vs `applyVisionCorrection()`

| | `setPose()` | `applyVisionCorrection()` |
|---|---|---|
| Purpose | Re-initialize (start of match, selector tap) | Nudge the running estimate |
| Effect | Hard set | Weighted blend toward the target |
| Resets source baselines? | **Yes** | **No** |
| Heading handling | Direct assignment | Shortest-path signed blend, wrap-safe |

`setPose()` resets every source's baseline **atomically with the pose change**, so the next
cycle can't compare a stale baseline against a fresh pose and produce a spurious jump.

`applyVisionCorrection(pose, weight)` blends by `weight` in `[0, 1]` — `1.0` is a full snap,
less nudges partway. It deliberately does **not** reset baselines: this is a correction to a
running estimate, not a re-initialization, so in-flight deltas since the last cycle are
preserved. Heading blends along the shortest signed path so a correction across the 0/360
boundary doesn't spin the pose the long way around.

Both take `resetMutex_`, which the fusion loop holds for a whole cycle — so a correction is
always fully applied between cycles, never halfway through one.

### Telemetry it publishes

| Channel | Type |
|---|---|
| `odom.pose.x`, `odom.pose.y`, `odom.pose.heading` | number |
| `odom.velocity.x`, `odom.velocity.y`, `odom.velocity.headingRate` | number |
| `odom.confidence` | text (`fullPod` / `partial` / `imeOnly`) |
| `odom.forwardPods.healthyCount`, `odom.strafePods.healthyCount` | integer |
| `hal.leftIme.healthy`, `hal.rightIme.healthy`, `hal.imu.healthy` | boolean |

---

## Configuration

[`include/lightspeed/odom/odometry_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/odom/odometry_constants.hpp)

```cpp
inline const TopologyConfig kOdometryTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods = {},                   // no tracking wheels on this robot
};

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 4.0,   // the robot's 4in drive omnis
    .gearRatio = 343.0 / 600.0,   // TODO: confirm the real external reduction
};
```

`gearRatio` is **wheel revolutions per motor-output-shaft revolution**. The same convention is
used by `motion::DrivetrainKinematicsConfig`, and the two must match.

---

## Adding tracking wheels later

If you build tracking pods:

1. Add their Rotation Sensor ports to `hal/config.hpp`.
2. Add a `PodConfig` per pod to `kOdometryTopology.pods` — name, role, measured `offsetInches`,
   and `ticksToInches` = wheel circumference / 36000.
3. In `initialize()`, construct a `hal::RotationSensor` and a `TrackingWheelSource` per pod, and
   pass the pointers into the `OdometryFusion` constructor instead of the empty vector.

Confidence will start reporting `fullPod` / `partial`, and the fallback path stops being used.

### The heading-redundancy warning

If no pod role has 2+ pods, `OdometryFusion` prints at startup:

```
[lightspeed::odom] WARNING: heading redundancy unavailable (no role has 2+ pods) --
IMU health is a hard dependency for heading in this topology.
```

**On this robot that warning is expected and correct** — with zero pods there is no differential
heading fallback, so if both IMUs fail, heading simply holds at its last value while position
keeps integrating. That is the honest failure mode, and it's why the message exists.

---

**Next:** [Subsystem Layer]({{ site.baseurl }}/layers/subsystem/) — running mechanisms.
