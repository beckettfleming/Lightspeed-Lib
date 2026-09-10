---
title: Tuning Guide
parent: Guides
nav_order: 2
permalink: /guides/tuning/
---

Every gain in Lightspeed ships as a placeholder. This page is the order to tune them in and how
to tell when each one is right.

> ⚠️ **Tune in this order.** Each stage depends on the one before it. Tuning turns before the
> velocity controller is right means retuning turns afterward.

---

## Before you start

1. **Ports are correct** and every motor spins the right way — [HAL Layer]({{ site.baseurl }}/layers/hal/).
2. **Physical measurements are entered** — track width, wheel diameter, gear ratio —
   [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/).
3. **You can see data.** Either:
   - **Diagnostic mode** (hold **Y** at boot) for a live 10 Hz CSV stream over USB, or
   - **The SD log** (`/usd/lslogNNN.csv`) reviewed after each run, or
   - **`pros terminal`** for the console prints.

   The serial stream is by far the best for tuning. See [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/).

4. **A charged battery.** Gains tuned on a sagging pack won't hold on a fresh one.
5. **Robot on blocks** for stages 1–2, wheels off the ground.

---

## Stage 1 — Drivetrain velocity controller

**File:** [`control/drivetrain_velocity_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/control/drivetrain_velocity_constants.hpp)
**Watch:** `drivetrain.left.targetRpm` vs `drivetrain.left.actualRpm`
**Robot:** on blocks, wheels free

This is the foundation. Everything else sits on top of it.

### 1a. `kV` — feedforward first, always

`kV` is millivolts per RPM of target. Get this right and PID barely has to do anything.

Start from theory: `12000 mV / (your max RPM)`. At 343 RPM that's ~35.

Then measure:

1. Set `kP = kI = kD = kS = 0`.
2. Command a mid-range velocity (say 150 RPM) and read `actualRpm`.
3. Adjust: `newKv = oldKv × (target / actual)`.
4. Repeat at 100, 200, and 300 RPM.

**Done when:** with PID off, actual velocity is within roughly 10% of target across the range.
That's most of the work finished.

### 1b. `kS` — static friction

`kS` is the millivolts needed just to start the wheels moving.

1. Command a very small velocity (~10 RPM).
2. If nothing moves, raise `kS` in steps of 50 until it just barely starts.

**Done when:** small commands produce motion. Too high and the robot lurches from a stop.

### 1c. `kP` — proportional trim

1. Raise `kP` from 0 until the robot responds crisply to step changes.
2. Back off ~20% from wherever it starts oscillating.

**Done when:** actual tracks target closely with no visible oscillation. If you need a large
`kP`, your `kV` is probably still too low — go back to 1a.

### 1d. `kD` — damping

Only if `kP` alone oscillates. Raise `kD` gradually until the oscillation damps out. Too much
makes the response sluggish and noisy.

### 1e. `kI` — usually leave at zero

Only add integral if there's a **persistent steady-state error** that `kV` and `kP` can't close.
If you do:

- Set `integralZone` to a small error band so it only integrates near the target.
- Keep `integralMax` (4000 mV) as the windup clamp.

Integral is the most common source of mysterious oscillation. Most VEX velocity loops don't need
it.

### 1f. `maxVoltageSlewRatePerSecond`

Caps how fast commanded voltage can change. Default 240000 mV/s = a full 12 V swing in ~50 ms.

- **Lower** if the drivetrain jerks or wheels slip on direction changes.
- **Raise** if the robot feels mushy and unresponsive.

### What good looks like

```
  target ----------________
        /                  \
       /   actual           \
      /  (lags slightly,     \___
     /    settles cleanly)
```

Bad: actual overshoots and rings (lower `kP`, raise `kD`), or never reaches target (raise `kV`).

---

## Stage 2 — Driver control feel

**File:** [`driver/driver_control_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/driver/driver_control_constants.hpp)
**Robot:** on the floor, with your actual driver

This stage is **subjective**. Your driver decides, not you.

| Knob | Try |
|---|---|
| `kDriveMode` | Have the driver try all three: `tank`, `arcade`, `curvature`. Curvature is smoother for long runs; arcade is more predictable. |
| `curveExponent` | 1.0 = linear. Raise toward 1.5–2.0 if the driver wants finer low-speed control. |
| `deadband` | Raise if the robot creeps with sticks centered. |
| `kMaxDriveRpm` | Must match your real top speed — see [Configuration Reference]({{ site.baseurl }}/reference/configuration/) §1. |
| `defaultMaxRpmPerSecond` | Lower if the robot feels twitchy or slips wheels. |

### Accel-limit rules

Once real mechanisms exist, add a rule per tipping risk:

```cpp
AccelLimitRule{.flagName = "lift.isExtended", .maxRpmPerSecond = 400.0},
```

Test by extending the mechanism and slamming the sticks. The robot should refuse to accelerate
hard. Watch for the console line:

```
[opcontrol] accel limit changed: 12000 -> 400 RPM/s (lift.isExtended=true)
```

---

## Stage 3 — Turn to heading

**Config:** `motion::kTurnToHeadingConfig`
**Watch:** `odom.pose.heading`
**Robot:** on the floor, clear space

### 3a. Verify track width first

Command a 90° turn. If the robot turns **the wrong amount** (not just overshooting — genuinely
70° or 110°), your `trackWidthInches` is wrong. Measure the real distance between left and right
wheel contact patches and fix it before touching any gain.

A gain will paper over a bad track width at one angle and fail at every other angle.

### 3b. `pidf.kP`

Deg/s of angular velocity per degree of error.

1. Raise from a small value until 90° turns complete promptly.
2. Back off when it starts overshooting and hunting.

### 3c. `pidf.kD`

Add damping if it overshoots and oscillates around the target.

### 3d. Settle conditions

- `settleTolerance` (2.0°) — how close counts as "there"
- `settleCycles` (8, ~160 ms) — how long it must stay there

Tighten the tolerance for precision; loosen it if turns take too long to declare victory.

### 3e. `timeoutSeconds`

Long enough for the largest turn you'll ever ask for, plus margin. Too short and turns get cut
off mid-motion.

**Test at multiple angles:** 15°, 90°, 180°. A gain tuned only at 90° often fails at 15°
(too slow) or 180° (overshoots).

---

## Stage 4 — Drive straight distance

**Config:** `motion::kDriveStraightDistanceConfig`
**Watch:** `odom.pose.x`, `odom.pose.y`, `odom.pose.heading`

### 4a. Verify odometry distance first

Command 24 inches. If it drives 20 or 28, the problem is **wheel diameter or gear ratio** in
`odom::kDriveImeConfig`, not a gain. Fix the geometry first.

### 4b. Motion profile limits

| Field | Meaning | Tune by |
|---|---|---|
| `maxVelocity` | in/s | Start conservative (~30). Raise until the robot can't keep up. |
| `maxAcceleration` | in/s² | Lower if wheels slip at the start. |
| `maxJerk` | in/s³ | Lower for gentler S-curve ramps; 0 disables S-curve entirely. |

### 4c. `distancePidf.kV` — keep near 1.0

This passes the profile's own velocity straight through as feedforward. It should stay at or very
near **1.0**. If you find yourself changing it a lot, something upstream (the velocity controller
or the unit conversion) is wrong.

### 4d. `distancePidf.kP` / `kD`

Trim only. Raise `kP` if the robot lags the profile; add `kD` if it oscillates around it.

### 4e. `headingCorrectionKP`

Deg/s of correction per degree of drift off the starting heading.

- Robot **curves** during a straight move → **raise** it.
- Robot **wobbles** side to side → **lower** it.

**Test both directions** — forward and backward — and at short (6 in) and long (72 in) distances.

---

## Stage 5 — Move to pose

**Config:** `motion::kMoveToPoseConfig`

Depends on stages 1, 3, and 4 being solid.

| Field | Symptom | Fix |
|---|---|---|
| `leadFraction` | Robot turns too late, arrives at the wrong heading | **Raise** (toward 0.6) |
| `leadFraction` | Robot swings out wide, path is inefficient | **Lower** (toward 0.2) |
| `positionToleranceInches` | Never settles | Loosen |
| `headingToleranceDegrees` | Never settles | Loosen |
| `timeoutSeconds` | Gets cut off mid-motion | Raise |

Test with the target pose **behind** the robot too — remember `MoveToPose` is forward-only and
won't reverse, so it will loop around. That's expected behavior, not a bug.

---

## Stage 6 — Pure pursuit

**Config:** `motion::kPurePursuitConfig`

The hardest to tune. Do it last.

### 6a. Path smoothing

| Field | Effect |
|---|---|
| `injectionSpacingInches` (2.0) | Point density. Smaller = smoother but more computation. |
| `weightSmooth` (0.25) | Higher = rounder corners, more deviation from your route. |
| `weightData` (0.5) | Higher = closer to your original waypoints, sharper corners. |

**`weightData + weightSmooth` must stay below 1.0** or smoothing won't converge.

Sanity-check a smoothed path by drawing it as a routine's `previewPoints` on the selector screen.

### 6b. Lookahead — the main knob

Start with `lookaheadSpeedGain = 0` and tune `minLookaheadInches` alone:

| Symptom | Fix |
|---|---|
| Robot **weaves/oscillates** along the path | **Raise** `minLookaheadInches` |
| Robot **cuts corners** badly | **Lower** `minLookaheadInches` |
| Robot misses the path entirely | Lower it a lot; check `trackWidthInches` |

Then add speed scaling: raise `lookaheadSpeedGain` so lookahead grows with speed (short and
accurate when slow, long and stable when fast). `maxLookaheadInches` caps it.

### 6c. Speed profile

Same fields as drive-straight, applied over total path length. Start slower than you think —
pure pursuit at high speed with a short lookahead is how robots end up in walls.

---

## Stage 7 — Vision (optional, last)

Only after everything else works and the sensor is physically mounted. Full procedure on
[Vision Layer]({{ site.baseurl }}/layers/vision/); it runs entirely through [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/).

---

## Reading the data

### From the SD log

```
/usd/lslog000.csv, lslog001.csv, ...   (one file per match)
```

Open in a spreadsheet and plot:

| Plot | Tells you |
|---|---|
| `drivetrain.left.targetRpm` vs `.actualRpm` | Velocity controller quality |
| `odom.pose.x` vs `odom.pose.y` | The actual path driven |
| `odom.pose.heading` over time | Turn behavior and straight-line drift |
| `odom.confidence` | Whether sensors dropped out mid-run |
| `drivetrain.batteryLow` | Whether a sagging pack explains bad behavior |

### From the console

```
[opcontrol] target=(150, 150)rpm slewed=(148, 148)rpm accelLimit=12000RPM/s arm=HOLDING
[purePursuit] pose=(24.3, 35.8, 91.2deg) lookahead=(36.0, 36.0) curvature=0.0123 speed=28.4
[moveToPose] pose=(41.2, 68.9, 84.1deg) target=(48.0, 72.0, 90.0deg) dist=7.5 curvature=0.0451
[ExampleArm] entering HOLDING
```

---

## Rules of thumb

- **Change one value at a time.** Two changes at once and you learn nothing.
- **Write down what you tried.** Including what didn't work.
- **Feedforward before feedback.** A well-tuned `kV` makes PID nearly unnecessary.
- **A fresh battery changes everything.** Re-verify on a charged pack.
- **If a gain has to be huge, something upstream is wrong.** Check units and geometry.
- **Test at multiple magnitudes.** Small, medium, large — for every primitive.
- **Geometry before gains.** Wrong track width or wheel diameter cannot be fixed by tuning.

---

**See also:** [Configuration Reference]({{ site.baseurl }}/reference/configuration/) · [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/) · [Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/)
