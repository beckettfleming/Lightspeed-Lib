# Getting started

## What you need

* **PROS CLI** — the project targets PROS kernel **4.2.2** with **liblvgl
  9.2.0** (see `project.pros`).
* **ARM GNU Toolchain 14.2.1** — what the project has been verified to compile
  clean against under `-Wall -Wextra`, zero warnings.
* A V5 Brain, a controller, and (eventually) a robot.

The project is a normal PROS project layout: `include/`, `src/`, `firmware/`,
`Makefile`, `common.mk`, `project.pros`. Build artifacts (`bin/`, `.d/`,
`.cache/`, `compile_commands.json`) are gitignored.

## Build and upload

```bash
pros build          # compile
pros upload         # build + upload to a connected brain
pros terminal       # tail stdout from the brain (printf output)
pros mu             # upload and open the terminal in one step
```

Lightspeed prints a fair amount to stdout — subsystem state transitions,
accel-limit changes, a 1 Hz driver-control heartbeat, autonomous routine
start/finish with the final pose. Keep `pros terminal` open while bench testing;
it is the cheapest diagnostic you have.

## First run, on a bench

The competition entry points behave like this out of the box:

**`initialize()`** builds every shared object, blocks for IMU calibration
(~2–3 s per IMU, and there are two), registers the demo routines, then starts
the selector GUI, the SD logger, and the dashboard.

> ⚠️ IMU calibration is blocking and happens here. Do not move the robot during
> `initialize()`.

**The brain screen** shows the auton selector: a schematic field with tappable
start locations. Tap one — this sets odometry's starting pose — then pick a
routine from the list beside the route preview, then tap **Confirm**.

**`autonomous()`** stops the selector (releasing the screen) and runs whichever
routine was confirmed. If the driver previewed a routine but never tapped
Confirm, it runs that one anyway and logs a warning — doing nothing for fifteen
seconds because someone forgot to tap a button is the worse failure.

**`opcontrol()`** runs the full driver pipeline every 20 ms:

```
joystick → InputProfile → DriveMode transform → accel-limited slew
         → DrivetrainVelocityController::setTargetVelocity()
```

with `R1`/`R2` moving the demo subsystem between presets, `L1` running a demo
button macro, and `Y` (held at boot, before this) entering diagnostic mode.

## Reading the codebase

Start with `src/main.cpp`. It is the wiring diagram: every object the library
provides is constructed there in dependency order, with comments explaining why
each one is where it is. Once you have read `main.cpp`, the layer pages in this
documentation will make sense in context.

Then read the `\file` comment at the top of any header you care about. They
explain design decisions, not just contents.

---

## Porting Lightspeed to a different robot

Lightspeed is written to be config-driven rather than hardcoded, so adopting it
on a different robot means editing constants files — not logic. Work through
these in order. Each step ends in something you can verify before moving on.

### 1. Port map — `include/lightspeed/hal/config.hpp`

This is the only file in the project that contains a raw port number. Update:

* `port::kLeftDrive*` / `port::kRightDrive*` — one entry per drive motor.
  Negative reverses that motor.
* `port::k*PodRotation` — one per tracking wheel you actually have.
* `port::kPrimaryImu` / `kSecondaryImu` — drop the secondary if you only run one
  IMU (see step 3).
* `kLeftDriveGroup` / `kRightDriveGroup` — set the correct `MotorGears`
  cartridge colour and the number of ports per side.
* Add a `MotorGroupConfig` for each real mechanism, replacing
  `kExampleArmGroup`.

**Verify:** upload and confirm nothing reports as disconnected. The dashboard
and the console will tell you.

### 2. Drivetrain geometry — `include/lightspeed/motion/motion_constants.hpp`

`kCherenkovKinematics` needs three real measurements:

* `trackWidthInches` — distance between the left and right wheel contact
  patches. Measure it; do not estimate from CAD unless the CAD is what got
  built.
* `wheelDiameterInches` — actual wheel diameter.
* `gearRatio` — wheel revolutions per motor-output-shaft revolution.

The same wheel diameter and gear ratio also appear in
`odom::kDriveImeConfig`, and the implied top speed appears in
`driver::kMaxDriveRpm` and `control::kDrivetrainVelocityConfig`'s `kV`. **All
four must agree.** See [Tuning § step 0](tuning.md#step-0--make-the-geometry-true).

### 3. Odometry topology — `include/lightspeed/odom/odometry_constants.hpp`

The fusion core handles **0–4 pods in any mix of roles**. Describe what you
have:

```cpp
inline const TopologyConfig kMyRobotTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods = {
        PodConfig{ .name = "leftForwardPod",  .role = PodRole::forward,
                   .offsetInches =  6.0, .ticksToInches = kTrackingWheelTicksToInches },
        PodConfig{ .name = "rightForwardPod", .role = PodRole::forward,
                   .offsetInches = -6.0, .ticksToInches = kTrackingWheelTicksToInches },
        PodConfig{ .name = "strafePod",       .role = PodRole::strafe,
                   .offsetInches =  0.0, .ticksToInches = kTrackingWheelTicksToInches },
    },
};
```

Also set `kTrackingWheelDiameterInches` to your actual tracking wheel.

**Zero pods is a supported configuration.** With an empty `pods` list, the
fusion core falls back to IME-derived forward distance plus IMU heading, and
reports `ConfidenceTier::imeOnly`. That is a legitimate starting point for a
robot without tracking wheels.

Then update `main.cpp` to construct one `hal::RotationSensor` and one
`TrackingWheelSource` per pod, **in the same order as the `pods` vector** —
`main.cpp` pairs them by index — and pass the resulting pointers to
`OdometryFusion`. For a single IMU, pass `nullptr` as `IMUSource`'s second
argument.

**Verify:** push the robot forward exactly 24 inches by hand and check
`odom.pose.y` on the dashboard. Then spin it 360° in place and check that x and
y come back to roughly where they started — persistent drift during a pure turn
means a `offsetInches` sign is flipped.

### 4. Subsystems

Delete `subsystem::demo::ExampleArm` and write real ones. See
[subsystem § Writing a subsystem](layers/subsystem.md#writing-a-subsystem) for
the pattern. Anything referencing the demo must be revisited:
`AutonomousContext::exampleArm`, `driver::demo::makeDemoArmCycleMacro`, the
`"exampleArm.isExtended"` rule in `kDriveAccelLimitConfig`, and `Dashboard`'s
fault indicator.

### 5. Driver feel — `include/lightspeed/driver/driver_control_constants.hpp`

`kDriveMode`, `kInputProfileConfig.curveExponent`, the deadband, and the
accel-limit rules are all driver-preference calls. Ask your driver; do not
inherit ours.

### 6. Field and routines

* `auton::kFieldDimensions` and `kFieldToScreenConfig` — confirm against the
  season field.
* `auton::kStartLocations` — replace the placeholders with your real legal
  starting positions and their poses.
* Replace `auton::demo::*` routines with real strategy.

### 7. Tune

Everything above is geometry and wiring. Now go to the
[tuning guide](tuning.md) and work through the gains in order.

---

## What you are inheriting

A realistic accounting, so nothing surprises you:

**Solid.** The layer separation, the fusion core's topology-agnostic design, the
single-instance wiring, the fail-safe paths (health-gated voltage, unconditional
fault stops, clamped outputs, timeouts on every blocking primitive), and the
telemetry plumbing. These were reviewed line by line and the review found and
fixed real bugs in them.

**Placeholder.** Nearly every numeric constant. See
[`BUCKET_B_CHECKLIST.md`](BUCKET_B_CHECKLIST.md).

**Untested on hardware.** Everything. The library has never been run on a
physical V5 brain with motors attached. It compiles clean and its logic has been
audited, which is not the same thing. Treat the first bench session as
integration testing, not tuning.

**Known limitations.**

* `MoveToPose` and `PurePursuitController` are **forward-only** — they steer
  toward a carrot point and never reverse, even if the target is behind the
  robot.
* `FlagRegistry` has no ownership enforcement: a second subsystem registering a
  colliding flag name and ignoring the `false` return can stomp the first. Use
  subsystem-prefixed names.
* `holonomic` kinematics is a recognized enum value with a stub fallback, not a
  working holonomic implementation.
* Fixed capacities: 32 telemetry channels, 16 flags, 8 subsystems, 4 pods per
  role. Exceeding one fails by dropping the extra, not by crashing — but check
  if you are near a limit.
