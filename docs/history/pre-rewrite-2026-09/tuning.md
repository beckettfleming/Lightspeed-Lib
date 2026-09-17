# Tuning guide

Nothing in this library has been tuned on hardware. Every gain is a placeholder.
This page is the order to fix that in.

**The order is not optional.** Each step assumes the ones above it are already
true. Tuning a distance PID on top of a wrong track width, or a velocity loop on
top of a wrong gear ratio, produces gains that compensate for the error — and
then break the moment you correct it.

## Before you start

* Keep `pros terminal` open. The library prints subsystem transitions,
  accel-limit changes, a 1 Hz driver-control heartbeat, and autonomous
  start/finish with the final pose.
* Put an SD card in the brain. The [SD logger](layers/telemetry.md#sdlogger)
  writes `/usd/lslogNNN.csv` with pose, velocity, target-vs-actual RPM, and
  health, at 25 Hz. Plotting a log after a run beats watching the screen during
  one.
* For live plotting, hold **Y** at boot to enter
  [diagnostic mode](layers/diagnostics.md), which starts the serial link — but
  note that diagnostic mode replaces normal operation, so it is for bench work,
  not driving.
* Have the robot on blocks with wheels free for the first two steps.

---

## Step 0 — Make the geometry true

Not tuning. Measurement. Do this first and do it properly; everything downstream
inherits these errors.

| Value | File | How to get it |
| --- | --- | --- |
| Motor ports, gearset | `hal/config.hpp` | From the actual wiring |
| `trackWidthInches` | `motion/motion_constants.hpp` | Measure between left/right wheel contact patches |
| `wheelDiameterInches` | `motion_constants.hpp`, `odom/odometry_constants.hpp` | Measure the wheel |
| `gearRatio` | same two files | Count teeth. Wheel revs per motor-shaft rev |
| `kTrackingWheelDiameterInches` | `odometry_constants.hpp` | Measure the tracking wheel — it is **not** the drive wheel |
| Pod `offsetInches` | `odometry_constants.hpp` | Measure each pod's lever arm from the tracking center |

### The four-way consistency check

The drivetrain's assumed top speed appears in four places and **they must
agree**:

1. `control::kDrivetrainVelocityConfig.pidf.kV` — sized as ≈ 12000 mV ÷ top RPM
2. `driver::kMaxDriveRpm`
3. `odom::kDriveImeConfig.gearRatio`
4. `motion::kCherenkovKinematics.gearRatio` and `wheelDiameterInches`

If your drivetrain is 600 RPM cartridges geared to 450 RPM output, then
`kMaxDriveRpm = 450`, both gear ratios are `450.0 / 600.0`, and `kV` starts at
about `12000 / 450 ≈ 26.7`. Get this wrong and every layer above will be tuned
around a lie.

### Verify

* Command a known voltage and confirm the reported RPM is plausible.
* Push the robot forward exactly 24 inches by hand; `odom.pose.y` should read
  ≈ 24.

---

## Step 1 — Drivetrain velocity loop

File: `include/lightspeed/control/drivetrain_velocity_constants.hpp`
Wheels **off the ground**.

This is the foundation. Every motion primitive commands velocity, so if velocity
tracking is poor, nothing above it can be made good.

Tune in this order:

**1. `kV` (velocity feedforward).** Zero `kP`, `kI`, `kD`, `kS`. Start at
`12000 / topRpm`. Command a mid-range target and compare
`drivetrain.left.actualRpm` to `.targetRpm`. Adjust `kV` until steady-state
actual lands close to target on its own. Feedforward should do most of the work;
feedback is trim.

**2. `kS` (static friction).** Command a very small target — 20–30 RPM. If the
wheels do not move at all, raise `kS` until they just break free. Too high and
the robot lurches at low speed.

**3. `kP`.** Raise until the response is snappy, back off at the first sign of
oscillation. Typical failure: a value large enough to buzz audibly at steady
state.

**4. `kD`.** Add a little to damp overshoot. Velocity measurements are noisy, so
`kD` here is usually small or zero.

**5. `kI`.** Usually leave at zero. If a steady-state offset persists that `kV`
cannot fix, add a small `kI` **and** set `integralZone` so it only integrates
near the target. `integralMax` (4000 mV) already caps the accumulator.

**6. `maxVoltageSlewRatePerSecond`.** 40000 mV/s is a full swing in ~300 ms.
Lower it if the drivetrain jerks; raise it if response feels mushy.

**Verify:** step from 0 to half speed and back. Plot target vs. actual from the
SD log. You want a fast rise, minimal overshoot, no oscillation, and a small
steady-state gap.

**On the ground:** re-check. Load changes things. Then let the battery drain and
confirm behavior stays consistent — that is the battery-sag compensation
working.

---

## Step 2 — Odometry

File: `include/lightspeed/odom/odometry_constants.hpp`
No gains here, only geometry — but geometry errors look exactly like bad tuning
later, so validate now.

**Straight-line test.** Push the robot forward exactly 24 inches by hand. Check
`odom.pose.y`. If it reads consistently long or short, `ticksToInches` (i.e. the
tracking wheel diameter) is wrong. A consistent *ratio* error is a measurement
error, not noise.

**Rotation test.** Spin the robot 360° in place. Heading should return to its
start. x and y should return to roughly where they started.

> If x/y **drift during a pure turn**, a pod's `offsetInches` sign or magnitude
> is wrong. This is the single most common odometry setup error. Flip the sign
> and repeat.

**Square test.** Drive a 4 × 4 ft square by hand and return to the start. Final
pose error tells you total accumulated drift. A few inches over that distance is
normal; a foot is not.

**Confidence check.** Unplug one tracking pod mid-run and confirm
`odom.confidence` degrades from `fullPod` to `partial`, and that pose stays
sane. Unplug all of them and confirm it drops to `imeOnly` and keeps tracking.

---

## Step 3 — Turn to heading

File: `include/lightspeed/motion/motion_constants.hpp` → `kTurnToHeadingConfig`

The output here is an **angular velocity command** in deg/s, so `kP` is "deg/s
of rotation commanded per degree of heading error."

* `kP` — raise until turns are brisk; back off on oscillation around the target.
* `kD` — add to damp overshoot. Turns overshoot more readily than straight
  moves, so `kD` matters more here.
* `kI` — leave at zero unless the robot consistently stops a degree or two short.
* `settleTolerance` / `settleCycles` — 2° for 8 cycles (~160 ms). Tighten only if
  the drivetrain can actually hold it; too tight and every turn burns its full
  timeout.
* `timeoutSeconds` — should be comfortably longer than a worst-case 180° turn,
  but short enough that a stuck robot does not eat the whole autonomous period.

**Verify:** 90°, 180°, and −90° turns. Each should settle without hunting.
Repeat five times and check the spread — repeatability matters more than any
single result.

---

## Step 4 — Drive straight

File: `motion_constants.hpp` → `kDriveStraightDistanceConfig`

Two loops here: distance tracking, and heading hold.

**Motion profile first.** `maxVelocity` (40 in/s), `maxAcceleration` (80 in/s²),
`maxJerk` (400 in/s³) describe what the drivetrain can actually do. Start
conservative. If the robot cannot keep up with the profile, the distance PID
will wind up trying.

**`distancePidf.kV` should stay at 1.0** — that passes the profile's own velocity
through as feedforward, which is the intent. Do not tune it away from 1.0 to
compensate for something else.

**`kP`** — "in/s of velocity commanded per inch of position error." Raise for
crisper arrival, back off on oscillation at the end of a move.

**`kD`** — damp the approach.

**`headingCorrectionKP`** — the P-only heading trim, in deg/s of correction per
degree of drift. Drive 6 ft and watch lateral deviation. Too low and the robot
curves away; too high and it weaves.

**Verify:** 12 in, 48 in, and a reverse move. Check final position error and
lateral deviation. Then drive 48 in five times and compare — consistency is what
autonomous depends on.

---

## Step 5 — Move to pose and pure pursuit

Both inherit everything above, so do not start here.

**`kMoveToPoseConfig.leadFraction`** (0.4) is the one genuinely new knob. Higher
= earlier, more aggressive turn-in. Lower = straighter approach with a sharper
final turn. Drive to a pose 4 ft away at 90° off your current heading and adjust
until the curve looks right and the heading tolerance is actually met.

**`kPurePursuitConfig` lookahead.** `min` 6 in, `max` 18 in,
`lookaheadSpeedGain` 0.3 in per in/s. Short lookahead tracks tightly and
oscillates; long lookahead is smooth and cuts corners. Tune on a path with both
a tight corner and a long straight.

**Path smoothing.** `injectionSpacingInches` 2.0, `weightData` 0.5,
`weightSmooth` 0.25. Keep `weightData + weightSmooth < 1` or smoothing will not
converge. More smoothing = rounder corners and more deviation from your intended
waypoints.

> Remember both primitives are **forward-only**. If a path needs the robot to
> back up, compose `TurnToHeading` and `DriveStraightDistance` instead.

---

## Step 6 — Driver control

File: `include/lightspeed/driver/driver_control_constants.hpp`

This is not engineering, it is preference. **Put your actual driver on the
controller** and change one thing at a time.

* `kDriveMode` — tank, arcade, or curvature. Let them try all three.
* `curveExponent` (2.0) — higher gives finer low-speed control at the cost of
  feeling sluggish off center.
* `deadband` (0.05) — just above whatever your controller's sticks drift to at
  rest.
* `kDriveAccelLimitConfig.defaultMaxRpmPerSecond` (2000) — effectively
  unlimited. Lower it if the robot tips or wheelies on hard acceleration.
* Accel-limit rules — one per mechanism whose extension should calm the drive.
  Watch the console: it prints a line each time the resolved limit changes, which
  is how you confirm the flag is actually toggling.

---

## Step 7 — Subsystems

Same PIDF class, position mode. Per subsystem:

* `kP` — raise until it reaches its target briskly.
* `kD` — damp overshoot. Mechanisms with mass overshoot; add `kD` before
  reducing `kP`.
* `kI` — only if gravity holds it short of the target, and then with an
  `integralZone`.
* `settleTolerance` / `settleCycles` — tight enough to be meaningful, loose
  enough to actually reach.
* Preset positions — measure them by driving the mechanism by hand and reading
  `getPositionRaw()`.

Check the fault paths too: stall the mechanism deliberately and confirm it stops
and enters your fault state rather than continuing to push.

---

## Step 8 — Vision

Only after everything above works. See
[vision § bringing vision online](layers/vision.md#bringing-vision-online) for
the full sequence — it starts with measuring the mount offset and ends with
verifying the skew formula's sign, and none of it should be trusted until the
bench harness agrees with a tape measure.

---

## Recording what you did

When a value stops being a placeholder, **tick it off in
[`BUCKET_B_CHECKLIST.md`](BUCKET_B_CHECKLIST.md)** and replace the `TODO`
comment above it with what you measured or how you tuned it:

```cpp
.kP = 18.5,   // tuned 2026-09-14 on blocks, oscillates above ~22
```

The next person to touch this — including you in four months — needs to know
which numbers are real.
