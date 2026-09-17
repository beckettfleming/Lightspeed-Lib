# `lightspeed::odom` — tracking the robot's position

This layer works out where the robot is on the field. Every **5 ms** it combines:

* the drive motors' built-in encoders (**IMEs**),
* one or two inertial sensors (**IMUs**), and
* 0 to 4 tracking wheels of each kind (**pods**),

into one **pose**: `x`, `y`, and heading.

The key design choice: **how many tracking wheels you have, and where, is a
setting — not code.** This robot might have none; the next might have three.
Odometry handles any setup, keeps working when sensors fail, and tells you how
much it trusts its answer.

New to odometry? See [Key ideas § The field and the robot's position](../concepts.md#the-field-and-the-robots-position).

---

## Types — `types.hpp`

### Pose and velocity

```cpp
struct Pose {
    double xInches = 0.0;
    double yInches = 0.0;
    double headingDegrees = 0.0;   // clockwise, 0–360
};

struct Velocity {
    double xInchesPerSecond = 0.0;
    double yInchesPerSecond = 0.0;
    double headingDegreesPerSecond = 0.0;
};
```

### Tracking wheel setup

```cpp
enum class PodRole : std::uint8_t { forward, strafe };

struct PodConfig {
    const char* name;
    PodRole     role;
    double      offsetInches;    // distance from tracking center (see below)
    double      ticksToInches;   // wheel circumference ÷ 36000 for a Rotation sensor
};

struct TopologyConfig {
    DrivetrainKinematics kinematics;   // tank (holonomic isn't implemented)
    std::vector<PodConfig> pods;
};
```

`PodConfig` has no port number on purpose — ports stay in `hal/config.hpp`. The
two are paired up in `main.cpp`.

### `offsetInches` — getting the sign right

The offset is how far the wheel is from the **tracking center**, measured
**sideways to the direction the wheel rolls**:

| Pod | Offset measures | Positive means |
| --- | --- | --- |
| Forward pod | how far left or right it is | **left** of center |
| Strafe pod | how far forward or back it is | **forward** of center |

Why it matters: when the robot spins in place, an off-center wheel still rolls.
Odometry uses the offset to subtract that rolling out, so a pure spin doesn't
look like the robot moved.

> ⚠️ The comment in `types.hpp` says forward pods are "positive = right". That's
> wrong — the math only works with positive = left, which is what
> `odometry_constants.hpp` already uses
> ([code review #15](../code-review.md#15-tracking-pod-offset-sign--the-code-comment-is-wrong)).
> A reversed sensor can flip things again, so **always do the spin test**: spin
> the robot in place and make sure `x`/`y` don't move. If they do, flip the sign.

### Confidence

```cpp
enum class ConfidenceTier : std::uint8_t { fullPod = 0, partial = 1, imeOnly = 2 };
const char* toString(ConfidenceTier tier);   // "FULL_POD", "PARTIAL", "IME_ONLY"
```

Listed from best to worst.

---

## Sources

A **source** reads one kind of sensor and reports **how much it changed since the
last read**, already converted to inches or degrees. Each has a baseline you can
reset.

### `IMESource`

Distance from a drive side's motor encoders. Build one for each side. Used when
there are no working forward tracking wheels.

```cpp
struct IMEConfig {
    double wheelDiameterInches;
    double gearRatio;   // wheel turns per motor turn
};

IMESource(hal::MotorGroup& motors, const IMEConfig& config);

double readDeltaInches();   // 0 if unhealthy, or on the first read
bool   isHealthy() const;
void   resetBaseline();
```

A stalled or hot motor still gives a good position, so those don't count as
unhealthy.

> ⚠️ But one unplugged motor makes the whole side "unhealthy," even though the
> others still work ([code review #18](../code-review.md#18-one-unplugged-drive-motor-turns-off-that-sides-encoder-input)).

### `IMUSource`

Heading change from one or two IMUs. If both are working, it averages them.

```cpp
explicit IMUSource(hal::Imu& primary, hal::Imu* secondary = nullptr);

double readHeadingDeltaDegrees();
bool   isHealthy() const;          // true if at least one IMU is ready
void   resetBaseline();
double getHeadingDegrees() const;  // 0–360, for display only
```

* For one IMU, pass `nullptr` as the second argument.
* Averaging assumes both IMUs were calibrated together.
* ⚠️ If one IMU drops out, heading can jump by half the difference between them
  ([code review #20](../code-review.md#20-the-dual-imu-heading-jumps-if-one-imu-drops-out)).

### `TrackingWheelSource`

Distance from one tracking wheel.

```cpp
TrackingWheelSource(hal::RotationSensor& sensor, const PodConfig& config);

double readDeltaInches();
bool   isHealthy() const;
void   resetBaseline();
const PodConfig& getConfig() const;
```

---

## `OdometryFusion`

Combines all the sources.

```cpp
OdometryFusion(DrivetrainKinematics kinematics,
               IMESource& leftIme, IMESource& rightIme, IMUSource& imu,
               const std::vector<TrackingWheelSource*>& pods);

Pose           getPose() const;
Velocity       getVelocity() const;
ConfidenceTier getConfidence() const;

void setPose(const Pose& pose);
void applyVisionCorrection(const Pose& visionPose, double confidenceWeight);
```

* It starts its own background task, so it can't be copied or moved.
* The pod objects must stay alive as long as `OdometryFusion` does.
* ⚠️ **At most 4 pods per role.** The code doesn't check; a fifth corrupts memory.

### What happens every 5 ms

1. **Read every sensor**, even ones it won't use, so their baselines stay current.
2. **Work out how much the robot turned.** It tries, in order:
   * the IMU,
   * the difference between the two farthest-apart working forward pods,
   * the same with strafe pods,
   * otherwise, assume no turn.
3. **Work out how far it moved** forward and sideways. For each working pod, it
   subtracts the rolling caused by turning (`distance − offset × turn`). Then:
   * 1 pod → use it,
   * 2 pods → average them,
   * 3 or more → take the middle value, so one slipping wheel gets outvoted.

   If no pods work for that direction: forward uses the average of the two drive
   sides; sideways is 0 (a tank drive can't slide sideways on its own).
4. **Correct for curved movement.** If the robot turned while moving, it traveled
   along an arc. The straight-line distance is slightly shorter, so it scales by
   `2·sin(turn/2) ÷ turn`.
5. **Convert to field directions** using the heading halfway through this step
   (more accurate than the heading at the start).
6. **Add it to the position**, update velocity and confidence, and record
   telemetry.

### Confidence

For each direction (forward and sideways):

* No pods set up, or none working → `IME_ONLY`
* All working → `FULL_POD`
* Some working → `PARTIAL`

The reported confidence is the **worse** of the two. Routines that need accuracy
can check `getConfidence()`, and it's shown on the dashboard.

> ⚠️ Odometry runs every 5 ms but most V5 sensors update every ~10 ms, so
> `getVelocity()` may be jumpy
> ([code review #21](../code-review.md#21-odometry-velocity-is-probably-jittery)).

### `setPose()` vs. `applyVisionCorrection()`

| | `setPose()` | `applyVisionCorrection()` |
| --- | --- | --- |
| Use it to | Set the starting position | Nudge a running position |
| Sensor baselines | Reset | Kept |
| Effect | Jumps straight to the new pose | Moves part of the way toward it |
| Called by | Selector, when a start location is tapped | Vision bench test |

`confidenceWeight` goes from 0 to 1. 1.0 jumps all the way; 0.5 goes halfway. The
partial move is on purpose — one bad camera reading shouldn't be able to teleport
the robot.

---

## Settings — `odometry_constants.hpp`

```cpp
inline constexpr double kTrackingWheelDiameterInches = 2.0;
inline constexpr double kTrackingWheelTicksToInches =
    (kPi * kTrackingWheelDiameterInches) / 36000.0;   // centidegrees → inches

inline const TopologyConfig kCherenkovTopology{ /* 2 forward pods + 1 strafe pod */ };

inline const IMEConfig kDriveImeConfig{
    .wheelDiameterInches = 4.0,
    .gearRatio = 343.0 / 600.0,
};
```

> ⚠️ Tracking wheel size, all pod offsets, and the gear ratio are placeholders.
> The 3-pod layout itself is a guess. See
> [placeholders § 2](../placeholders.md#2-odometry-measurements) and the
> [porting guide](../porting.md#step-3--tracking-wheels-and-inertial-sensors).

**Order matters.** `main.cpp` pairs `kCherenkovTopology.pods[0]` with the first
Rotation sensor it builds, `pods[1]` with the second, and so on. If you reorder
one, reorder the other.

---

## Telemetry channels

| Channel | Type |
| --- | --- |
| `odom.pose.x`, `odom.pose.y`, `odom.pose.heading` | number |
| `odom.velocity.x`, `odom.velocity.y`, `odom.velocity.headingRate` | number |
| `odom.confidence` | text |
| `odom.forwardPods.healthyCount`, `odom.strafePods.healthyCount` | integer |
| `hal.leftIme.healthy`, `hal.rightIme.healthy`, `hal.imu.healthy` | true/false |
