---
title: Troubleshooting
parent: Guides
nav_order: 3
permalink: /guides/troubleshooting/
---

Symptoms, likely causes, and where to look. Ordered roughly by how early you'd hit them.

---

## Build and upload

### Warnings appear in the build

This project maintains a **zero-warning build** under `-Wall -Wextra`. A warning means something
you changed introduced it. Fix it rather than ignoring it — that's the standard here.

### A header isn't found

Include paths are absolute from the include root:

```cpp
#include "lightspeed/motion/pure_pursuit_controller.hpp"   // correct
#include "../motion/pure_pursuit_controller.hpp"           // wrong
```

### Changes don't seem to take effect

Try a clean rebuild — hot/cold linking occasionally needs it:

```bash
pros make clean && pros make && pros upload
```

---

## Startup

### The robot does nothing for 2–3 seconds at boot

**Expected.** `initialize()` calls `Imu::calibrate(true)` on both IMUs, which blocks. Keep the
robot still during it.

### Nothing happens at all — no GUI, no driver control

Did you hold **Y** while powering on? That enters [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/), which never returns to
normal operation. Power-cycle without holding Y.

### Console shows a heading-redundancy warning

```
[lightspeed::odom] WARNING: heading redundancy unavailable (no role has 2+ pods)
```

**Expected on this robot.** It has no tracking pods, so there's no differential-heading fallback
if both IMUs fail. The message is honest, not an error. See [Odometry Layer]({{ site.baseurl }}/layers/odometry/).

---

## Driving

### The robot drives backward when I push the stick forward

Two candidates, in order:

1. **Motor direction.** Check the port signs in `hal/config.hpp`. A negative port reverses that
   motor. Both sides need to produce forward motion from positive voltage.
2. **The joystick sign convention.** `main.cpp` negates `ANALOG_LEFT_Y` and `ANALOG_RIGHT_X` when
   normalizing:

   ```cpp
   .leftY  = -normalizeStick(master.get_analog(ANALOG_LEFT_Y)),
   .rightX = -normalizeStick(master.get_analog(ANALOG_RIGHT_X)),
   ```

   Whether that's right depends on your motor orientation. **Confirm on the real robot** and flip
   whichever sign is wrong. Fix the motor ports first — that's the more fundamental of the two.

### The robot spins instead of driving forward

One side is reversed relative to the other. Negate (or un-negate) that side's ports in
`hal/config.hpp`.

### Turning goes the wrong way

The turn axis sign. Check `rightX`'s negation in `main.cpp`, and confirm heading is
clockwise-positive (see [Coordinate System]({{ site.baseurl }}/reference/coordinates/)).

### The robot is sluggish, or won't hit full speed

| Check | Fix |
|---|---|
| `kV` too low | Retune — [Tuning Guide]({{ site.baseurl }}/guides/tuning/) stage 1a |
| `kMaxDriveRpm` too low | It must match your real top speed |
| An accel-limit rule is active | Watch for the `accel limit changed` console line |
| Battery low | Check `drivetrain.batteryLow` in telemetry |
| A motor is disconnected | Check `drivetrain.left.connectedMotors` |

### The robot jerks or slips wheels

Lower `maxVoltageSlewRatePerSecond`, or lower `defaultMaxRpmPerSecond` in the driver config.

### One side stops working mid-match

Check `drivetrain.left.health` / `.right.health` in the SD log:

| Value | Meaning |
|---|---|
| `STALLED` | Jammed or badly overloaded — the controller zeroed output deliberately |
| `OVER_TEMP` | Motor thermal protection — output zeroed |
| `DISCONNECTED` | Check `connectedMotors`; if 0, the side is fully dead |

A **partial** disconnect keeps driving in a degraded state by design — a side with one working
motor shouldn't stop entirely.

### The drivetrain oscillates

`kP` is too high or `kI` is nonzero. Lower `kP` by ~20%, add `kD`, and set `kI = 0` unless you
specifically needed it.

---

## Odometry

### The pose doesn't change when I push the robot

| Check | How |
|---|---|
| IMU healthy? | `hal.imu.healthy` — false means calibration failed or the port is wrong |
| Encoders healthy? | `hal.leftIme.healthy` / `hal.rightIme.healthy` |
| Ports correct? | `hal/config.hpp` |

### The pose changes by the wrong amount

Distance conversion. Push the robot exactly 24 inches and compare:

```
scaleFactor = actualInches / reportedInches
```

Then fix `wheelDiameterInches` or `gearRatio` in `odom::kDriveImeConfig` by that factor. **Both
must also match `motion::kDrivetrainKinematics`** — see [Configuration Reference]({{ site.baseurl }}/reference/configuration/).

### Heading drifts while sitting still

Normal IMU behavior to a small degree. If it's severe:

- Was the robot moved during calibration? Re-calibrate while stationary.
- Is one IMU faulty? `IMUSource` averages both — one bad sensor pollutes the average.

### x/y drift during a pure in-place rotation

The classic geometry bug. In order of likelihood:

1. **`trackWidthInches` is wrong** — measure it again.
2. If you have tracking pods: **a `PodConfig::offsetInches` sign is flipped.**
3. The IMU is reporting the wrong sign of rotation.

### Heading jumps by ~360°

Something is doing delta math on the **wrapped** heading. Use
`Imu::getContinuousHeadingDegrees()` for any subtraction, and
`motion::headingErrorDegrees()` for any comparison. See [Coordinate System]({{ site.baseurl }}/reference/coordinates/).

### Confidence always reads `imeOnly`

**Expected on this robot** — with zero tracking pods configured, the fallback path is always
used, and `imeOnly` is the honest tier. It becomes meaningful only if you add pods.

---

## Autonomous

### Nothing runs during autonomous

Check the console:

```
[autonomous] no routine confirmed or previewed via the GUI selector -- nothing to run
```

Nothing was ever selected. If a routine was *previewed* but not confirmed, it runs anyway with a
warning — see [Autonomous Layer]({{ site.baseurl }}/layers/autonomous/).

### The routine runs but goes to the wrong place

**Did you pick a start location on screen 1?** Tapping it calls `odometry.setPose()`. Without
that, odometry starts at `(0, 0, 0)` and every field-coordinate move goes somewhere else
entirely.

This is the single most common autonomous failure.

### The routine stops partway through

A primitive hit its timeout. Watch the console for which one stopped printing, then:

- Raise that primitive's `timeoutSeconds`, or
- Loosen its settle tolerance, or
- Fix the tuning so it actually converges — [Tuning Guide]({{ site.baseurl }}/guides/tuning/)

### A primitive never settles

Its tolerance is tighter than the robot can achieve. Loosen `settleTolerance` /
`positionToleranceInches` / `headingToleranceDegrees`, or improve the tuning.

### Pure pursuit weaves along the path

Lookahead too short. Raise `minLookaheadInches`.

### Pure pursuit cuts corners

Lookahead too long. Lower `minLookaheadInches`, or lower `weightSmooth` so smoothing rounds the
corners less.

### `MoveToPose` loops around instead of backing up

**Expected.** It's forward-only by design and won't reverse. Use `TurnToHeading` +
`DriveStraightDistance` with a negative distance if you need to back up. See [Motion Layer]({{ site.baseurl }}/layers/motion/).

---

## Subsystems

### A subsystem doesn't move

| Check | |
|---|---|
| Was `Scheduler::instance().start()` called? | In `initialize()`, after constructing subsystems |
| Is it in a fault state? | Watch for `[Name] entering FAULTED` on the console |
| Motor health? | A stall or over-temp zeroes output by design |
| Is the state machine in `idle`? | `idle` writes 0 V |

### A preset does nothing

```
[ExampleArm] WARNING: unknown preset 'HIGHT'
```

A typo. Preset names are case-sensitive string literals matched with `strcmp`.

### A flag isn't affecting the accel limit

1. Is it registered? `registerFlag()` returns `false` on a duplicate name or a full registry.
2. Does the name in `AccelLimitRule::flagName` match **exactly**?
3. Is it actually being set? It appears in the SD log automatically.
4. Watch for the console line: `[opcontrol] accel limit changed: ...`

### More than 8 subsystems

`Scheduler::kMaxSubsystems` is 8. Raise it in `scheduler.hpp`.

### More than 16 flags

`FlagRegistry::kMaxFlags` is 16. Raise it in `flag_registry.hpp`.

---

## Telemetry

### No SD log file appears

| Check | |
|---|---|
| Card inserted and FAT32? | |
| Console message? | `[SdLogger] logging to '/usd/lslog000.csv'` on success |
| Did the match start? | A file opens on the **disabled → enabled** transition, not at boot |

### Logging stopped mid-match

The card was pulled or failed. The `fwrite` return is checked and logging stops cleanly rather
than retrying a dead file. That batch of rows is lost.

### A channel is missing from the log

- **32-channel cap** (`TelemetryBus::kMaxChannels`) — an over-limit channel is dropped.
- The channel registered **after** the CSV header was written, so rows are clamped to the header
  width. Record it earlier, or restart the log.

### The dashboard doesn't appear

It only draws during **driver control** (`!is_disabled() && !is_autonomous()`). Before the match
the selector GUI owns the screen; during autonomous nothing draws. That's by design.

---

## Vision

### No tags detected

| Check | |
|---|---|
| Sensor port correct? | `hal/config.hpp` |
| Right tag family? | `kAiVisionSensor.tagFamily` (currently `tag_16H5`) |
| Lighting and distance | Get closer, add light |
| Tag printed at the right size? | Must match `tagSizeInches` |

### Tags detected but always rejected

Run [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/) and read the `GateReason`:

| Reason | Meaning | Fix |
|---|---|---|
| `unknownTagId` | Not in `kTagWorldMap` | Add it |
| `notStableYet` | Fewer consecutive frames than the threshold | Hold steadier, or lower `stableFrameThreshold` |
| `skewTooHigh` | Tag too angled | Approach squarer, or raise `maxSkewDegrees` |
| `noTagDetected` | See above | |

### Corrections push the pose the wrong way

Most likely candidates, in order:

1. **The skew formula's sign.** Known to need on-hardware verification — see
   [`tag_pose_solver.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/vision/tag_pose_solver.cpp) and [Vision Layer]({{ site.baseurl }}/layers/vision/).
2. **`kAiVisionMountOffset`** is wrong. A 6-inch error here is a 6-inch error in every correction.
3. **`kTagWorldMap`** headings — the tag's heading is its face normal's **outward** direction.

---

## Anything else

1. **Read the console.** Nearly everything in this project prints when it does something
   interesting.
2. **Read the SD log.** Plot target vs. actual, and the pose over time.
3. **Read the header comment** for whatever component is misbehaving — every header explains its
   own design decisions and documented limitations in detail.
4. **Check [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/)** — the value you're fighting may still be a placeholder.

---

**See also:** [Tuning Guide]({{ site.baseurl }}/guides/tuning/) · [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/) · [Coordinate System]({{ site.baseurl }}/reference/coordinates/)
