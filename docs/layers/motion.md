---
title: Motion
parent: Layers
nav_order: 6
permalink: /layers/motion/
---

**Namespace:** `lightspeed::motion` · **Headers:** `include/lightspeed/motion/`

Five motion primitives that move the robot to a place, plus the shared math they're built on.
Every one of them reads odometry for feedback and ends by calling
`DrivetrainVelocityController::setTargetVelocity()` — no primitive writes voltage directly.

---

## Choosing a primitive

| Primitive | Moves to | Heading control | Style | Blocking |
|---|---|---|---|---|
| `TurnToHeading` | a heading | that's the point | rotate in place | yes |
| `DriveStraightDistance` | a distance forward/back | holds the starting heading | straight line | yes |
| `DriveToPoint` | `(x, y)` | none at arrival | turn, then drive | yes |
| `MoveToPose` | `(x, y, heading)` | **yes** | one continuous curve | yes |
| `PurePursuitController` | along a waypoint path | none at arrival | smooth path following | yes |

```
  Just need to face a direction?          -> TurnToHeading
  Just need to go forward N inches?       -> DriveStraightDistance
  Need to reach a point, don't care
    which way you're facing there?        -> DriveToPoint
  Need to reach a point FACING a
    specific direction?                   -> MoveToPose
  Need to follow a specific route
    around obstacles?                     -> PurePursuitController
```

**All five are blocking** — they run their own loop until settled or timed out, then command
`(0, 0)` before returning. Call them from the autonomous task, never from `opcontrol()`.

**All five have both a settle condition and a timeout.** A primitive that can't reach its target
gives up rather than hanging the routine forever.

---

## Shared math

### `DrivetrainKinematics`

Tank-drive geometry and unit conversion used by every primitive.

```cpp
struct DrivetrainKinematicsConfig {
    double trackWidthInches;     // distance between left and right wheel contact patches
    double wheelDiameterInches;
    double gearRatio;            // wheel revs per motor-output-shaft rev
};
```

| Function | Converts |
|---|---|
| `inchesPerSecondToRpm()` | Wheel surface speed to motor RPM |
| `rpmToInchesPerSecond()` | The reverse |
| `curvatureToWheelSpeeds(forward, curvature, trackWidth)` | A curved path into left/right speeds |
| `angularVelocityToWheelSpeeds(forward, degPerSec, trackWidth)` | A rotation rate into left/right speeds |

**Curvature** is `1/inches` — the reciprocal of the turn radius. Positive curves right. Zero is
straight. `angularVelocityToWheelSpeeds` is used by turn-to-heading (forward = 0) and by
drive-straight's heading-hold trim (angular velocity = a small correction).

`trackWidthInches` is the parameter most likely to be wrong on a new robot, and it directly
scales every turn. Measure it carefully.

### `PoseMath`

```cpp
LocalOffset toLocalFrame(const odom::Pose& robotPose, double targetX, double targetY);
double distanceToPoint(const odom::Pose& robotPose, double targetX, double targetY);
double headingToPoint(const odom::Pose& robotPose, double targetX, double targetY);
double headingErrorDegrees(double from, double to);   // shortest signed error, (-180, 180]
```

`headingErrorDegrees()` is the one to reach for whenever you compare headings. It handles the
0/360 wrap correctly — positive means "rotate clockwise". Comparing headings by subtraction will
eventually make your robot spin 350° the wrong way.

### `MotionProfile`

A generic, **unit-agnostic** 1-D time-parameterized profile. Works in inches for the drivetrain,
and would work in degrees for a mechanism.

```cpp
struct MotionProfileConfig {
    double maxVelocity;
    double maxAcceleration;
    double maxJerk = 0.0;   // <= 0 disables S-curve
};

MotionProfile profile(0.0, 48.0, config);
MotionState state = profile.sample(elapsedSeconds);   // .position, .velocity, .acceleration
double total = profile.getTotalDuration();
```

**Trapezoidal** (`maxJerk <= 0`): accelerate at a constant rate, cruise, decelerate. Simple, and
what most VEX code does.

**S-curve** (`maxJerk > 0`): acceleration itself ramps up and down, so the robot doesn't jerk at
the start and end of a move. Gentler on the drivetrain and less likely to slip wheels.

The S-curve is used **only when the move is long enough** to reach both max acceleration and max
velocity with jerk-limited ramps. Short moves automatically fall back to trapezoidal — an
S-curve that never reaches its cruise phase isn't meaningful.

`sample(t)` clamps `t` to `[0, getTotalDuration()]`, so sampling past the end returns the final
state.

### `Path`

Turns a handful of hand-placed waypoints into a dense, smoothed point list for pure pursuit.

```cpp
struct Waypoint { double x; double y; };

struct PathSmoothingConfig {
    double injectionSpacingInches;      // spacing of injected points along each segment
    double weightData;                  // pull toward the original injected position
    double weightSmooth;                // pull toward neighbors' average
    double smoothingToleranceInches;    // stop once the largest per-point move is below this
    std::uint32_t maxSmoothingIterations;
};

std::vector<Waypoint> path = buildSmoothPath(rawWaypoints, config);
```

Two phases:

1. **Injection** — evenly spaced points along each straight segment.
2. **Gradient-descent smoothing** — iteratively pull each interior point toward its neighbors'
   average (`weightSmooth`) while keeping it near where it was injected (`weightData`).

Endpoints never move. Fewer than 2 raw waypoints returns the input unchanged.

**Keep `weightData + weightSmooth < 1`** or the smoothing won't converge. Higher `weightSmooth`
means rounder corners but more deviation from your intended route.

This is the standard lightweight alternative to true splines at VEX scale.

---

## The primitives

### `TurnToHeading`

```cpp
motion::TurnToHeading turn(drivetrain, odometry, motion::kTurnToHeadingConfig);
turn.run(90.0);   // rotate in place to face 90 degrees
```

A PIDF controller on heading error. Its output is an **angular velocity command** (deg/s),
converted to left/right wheel speeds with forward speed = 0 and fed to the drivetrain.

`kV`/`kA`/`kS` are normally 0 here — there's no feedforward target velocity for a static
heading target. Settle condition is `pidf.settleTolerance` (2°) for `pidf.settleCycles` (8
cycles at 50 Hz, so ~160 ms).

### `DriveStraightDistance`

```cpp
motion::DriveStraightDistance drive(drivetrain, odometry, motion::kDriveStraightDistanceConfig);
drive.run(24.0);    // 24 inches forward
drive.run(-12.0);   // 12 inches backward
```

Four things working together:

1. A `MotionProfile` provides the position/velocity/acceleration envelope.
2. A PIDF tracks profiled position against measured forward distance.
3. A **P-only heading trim** holds the heading captured at the start of the call, so the move
   doesn't drift off-line.
4. Output goes to the drivetrain velocity controllers.

`distancePidf.kV` should normally be **~1.0** — that passes the profile's own velocity straight
through as feedforward, with kP/kI/kD only providing trim. That's the key idea: the profile does
the driving, PID just corrects.

`headingCorrectionKP` is deg/s of correction per degree of drift. Raise it if the robot curves;
lower it if it wobbles.

### `DriveToPoint`

```cpp
motion::DriveToPoint driveTo(turnToHeading, driveStraightDistance, odometry);

driveTo.turnToPoint(48.0, 72.0);    // just face it
driveTo.driveToPoint(48.0, 72.0);   // face it, then drive to it
```

**Pure composition — no new control loop.** It computes the heading and distance to the target
from the current pose, then calls `TurnToHeading` followed by `DriveStraightDistance`.

Consequence: it's a **discrete turn-then-drive**, and it has **no heading target at arrival**.
The robot ends up at the point facing whatever direction the approach left it. If you need a
specific arrival heading, use `MoveToPose`.

### `MoveToPose` (boomerang)

```cpp
motion::MoveToPose moveTo(drivetrain, odometry, motion::kMoveToPoseConfig);
moveTo.run(48.0, 72.0, 90.0);   // arrive at (48,72) FACING 90 degrees
```

Drives to a full `(x, y, heading)` pose in **one continuous curved motion**.

#### How the boomerang trick works

```
                      carrot point
                           *
                          /|
                         / |  leadFraction * remainingDistance,
                        /  |  offset BEHIND the target along the
                       /   |  TARGET's own heading
                      /    |
     robot  ------>  /     v
       O                   T  target pose (x, y, heading)
                           ^
                           | target heading
```

Each cycle it computes a **carrot point**: the target, walked backward along **the target's own
heading** by `leadFraction × remaining distance`. The robot steers toward the carrot using the
same curvature math as pure pursuit.

Two properties fall out for free:

- As the robot closes in, remaining distance shrinks, so **the carrot converges onto the target**.
- Because the offset direction is fixed to the **target's** heading (not the robot's), chasing
  the carrot naturally lines the robot up with that heading on arrival — **no separate
  heading-blend term is needed.**

`leadFraction` (0–1) controls the shape: **higher = earlier, more aggressive turn-in; lower =
straighter approach with a sharper final turn.** Currently 0.4.

Settles when **both** position tolerance *and* heading tolerance are met for `settleCycles`
consecutive cycles.

> **Known limitation (documented, not a bug):** like pure pursuit, `MoveToPose` is
> **forward-only**. It always drives toward the carrot rather than reversing, even if the target
> is behind the robot. A target that genuinely requires backing up isn't handled specially.

### `PurePursuitController`

```cpp
motion::PurePursuitController pursuit(drivetrain, odometry, motion::kPurePursuitConfig);

pursuit.follow({
    {12.0, 12.0},
    {24.0, 36.0},
    {48.0, 36.0},
});
```

Smooths the raw waypoints (via `buildSmoothPath`), then follows the result.

#### Each cycle

```
  1. Find the closest path point to the robot (searching forward only,
     so the robot can never snap backward onto an earlier part of the path)
  2. Compute an ADAPTIVE lookahead distance:
        lookahead = clamp(minLookahead + speedGain * currentSpeed,
                          minLookahead, maxLookahead)
  3. Find the first path point at or beyond that distance
     (falls back to the final point when the robot is close to the goal)
  4. Transform it into the robot's local frame
  5. curvature = 2 * strafeRight / distanceSquared
  6. Sample the motion profile for the target speed
  7. curvatureToWheelSpeeds() -> setTargetVelocity()
  8. Settled when within positionTolerance of the END point for settleCycles
```

**Adaptive lookahead** is the tuning knob that matters most:

| Lookahead | Behavior |
|---|---|
| Too **short** | Robot oscillates and weaves, chasing points it's nearly on top of |
| Too **long** | Robot cuts corners badly and may miss the path entirely |
| **Speed-scaled** | Short at low speed for accuracy, long at high speed for stability |

Start with `lookaheadSpeedGain = 0` and tune the fixed minimum first, then add the speed term.

Like `MoveToPose`, this is forward-only. It's a no-op with fewer than 2 waypoints.

---

## Configuration

[`include/lightspeed/motion/motion_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/motion/motion_constants.hpp)

Shared geometry, used by every primitive:

```cpp
inline const DrivetrainKinematicsConfig kDrivetrainKinematics{
    .trackWidthInches = 12.0,     // TODO: confirm once the drivetrain is built
    .wheelDiameterInches = 4.0,
    .gearRatio = 343.0 / 600.0,
};
```

> `wheelDiameterInches` and `gearRatio` must match `odom::kDriveImeConfig`, and the implied top
> speed must match `driver::kMaxDriveRpm` and the drivetrain's `kV`. All four move together —
> see [Configuration Reference]({{ site.baseurl }}/reference/configuration/).

Per-primitive configs (`kTurnToHeadingConfig`, `kDriveStraightDistanceConfig`,
`kMoveToPoseConfig`, `kPurePursuitConfig`) live in the same file. **Every gain in all of them is
a placeholder** — see [Tuning Guide]({{ site.baseurl }}/guides/tuning/) for the order to tune them in.

All four primitives run their loop at **50 Hz** (`loopPeriodMs = 20`), which is plenty for
odometry-feedback motion; the 100 Hz velocity controller underneath does the fast work.

---

## Console output

Every primitive prints progress roughly every 250 ms:

```
[purePursuit] pose=(24.3, 35.8, 91.2deg) lookahead=(36.0, 36.0) curvature=0.0123 speed=28.4
[moveToPose] pose=(41.2, 68.9, 84.1deg) target=(48.0, 72.0, 90.0deg) dist=7.5 curvature=0.0451
```

Watch these while tuning — they're the fastest way to see whether a primitive is converging,
oscillating, or timing out.

---

**Next:** [Autonomous Layer]({{ site.baseurl }}/layers/autonomous/) — picking and running routines.
