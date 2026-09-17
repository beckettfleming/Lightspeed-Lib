# Porting guide: putting Lightspeed on your robot

This guide takes Lightspeed from its placeholder setup to your actual robot.
Work through the steps **in order**. Each one ends with a check so you know it's
right before moving on.

If you just want the robot moving as fast as possible, use
[easy mode](quick-start.md#easy-mode) first and come back here later.

---

## Step 1 — Ports

**File:** `include/lightspeed/hal/config.hpp`

This is the only file with port numbers. In the `port` section:

* **Drive motors** — `kLeftDriveFront`, `kLeftDriveMiddle`, `kLeftDriveBack`,
  and the same for the right. Use a negative number to reverse a motor.
* **Tracking wheels** — `kLeftForwardPodRotation`, `kRightForwardPodRotation`,
  `kStrafePodRotation`.
* **Inertial sensors** — `kPrimaryImu`, `kSecondaryImu`.
* **AI Vision sensor** — `kAiVisionSensor`.

Below the port list:

* In `kLeftDriveGroup` / `kRightDriveGroup`, set the cartridge color
  (`MotorGears::blue`, `green`, or `red`) and list the right number of motors.
  If you have 2 motors per side, delete one port from each list (and its
  constant).
* Replace `kExampleArmGroup` with a `MotorGroupConfig` for each real mechanism.

**Check:** upload and look at the terminal. The drivetrain prints `DISCONNECTED`
health when a motor is missing, and the dashboard shows it too.

---

## Step 2 — Speed and geometry

These numbers describe your drivetrain. They need to be **measured**, and several
of them appear in more than one file, so they have to **match**.

### Speed units

The drivetrain controller works in **motor RPM** (the motor's own shaft speed:
600 for blue, 200 green, 100 red).

| Setting | File | Set to |
| --- | --- | --- |
| `kMaxDriveRpm` | `driver/driver_control_constants.hpp` | Your cartridge RPM (e.g. 600) |
| `.kV` | `control/drivetrain_velocity_constants.hpp` | Start at 12000 ÷ cartridge RPM (e.g. 20) |

> The shipped values (343 and 35) assume wheel RPM, which is wrong. See
> [code review #1](code-review.md#1-drivetrain-speed-units-disagree--driver-control-is-capped-at-57-speed).

### Wheels and gears

Set these the same in **both** `motion/motion_constants.hpp`
(`kCherenkovKinematics`) and `odom/odometry_constants.hpp` (`kDriveImeConfig`):

* `wheelDiameterInches` — measure the actual wheel.
* `gearRatio` — wheel turns per motor turn. Count gear teeth: motor-side gear
  teeth ÷ wheel-side gear teeth. Direct drive is `1.0`.

In `motion_constants.hpp` only:

* `trackWidthInches` — distance between where the left and right wheels touch
  the ground. **Measure the real robot**; don't trust CAD unless the robot
  matches it exactly.

**Check:** with the robot on the ground, push it forward exactly 24 inches by
hand and watch the dashboard's position. With no tracking wheels, `y` should
read close to 24.

---

## Step 3 — Tracking wheels and inertial sensors

**File:** `include/lightspeed/odom/odometry_constants.hpp`, plus `src/main.cpp`

### Describe your tracking wheels

Odometry works with **0 to 4 tracking wheels of each kind**, in any mix.
Describe what you have in the topology:

```cpp
inline const TopologyConfig kCherenkovTopology{
    .kinematics = DrivetrainKinematics::tank,
    .pods = {
        PodConfig{ .name = "leftForwardPod",  .role = PodRole::forward,
                   .offsetInches =  6.0,   // 6 in LEFT of center
                   .ticksToInches = kTrackingWheelTicksToInches },
        PodConfig{ .name = "rightForwardPod", .role = PodRole::forward,
                   .offsetInches = -6.0,   // 6 in RIGHT of center
                   .ticksToInches = kTrackingWheelTicksToInches },
        PodConfig{ .name = "strafePod",       .role = PodRole::strafe,
                   .offsetInches =  0.0,   // + = forward of center
                   .ticksToInches = kTrackingWheelTicksToInches },
    },
};
```

* `role` — `forward` if it rolls when driving forward, `strafe` if it rolls
  when sliding sideways.
* `offsetInches` — distance from the tracking center, measured sideways to the
  way the wheel rolls. **Forward pods: positive = left. Strafe pods: positive =
  forward.**
* Set `kTrackingWheelDiameterInches` to your tracking wheel's size (it's usually
  smaller than the drive wheels).
* Don't put more than 4 pods of one role — the code doesn't check, and a fifth
  will corrupt memory.

### Match `main.cpp` to it

`main.cpp` builds one Rotation sensor and one `TrackingWheelSource` for each pod,
and **pairs them by position in the list**. `pods[0]` goes with the first sensor,
`pods[1]` with the second, and so on. If you add, remove, or reorder pods, do the
same in `initialize()`.

### No tracking wheels

Zero pods is a supported setup. Odometry uses the drive motor encoders for
distance and the IMU for heading, and reports `IME_ONLY` confidence.

1. In `odometry_constants.hpp`, make the list empty: `.pods = {},`
2. In `main.cpp`, delete the lines that build `gLeftForwardRotation`,
   `gRightForwardRotation`, `gStrafeRotation`, `gLeftForwardPod`,
   `gRightForwardPod`, and `gStrafePod`.
3. Pass an empty list to odometry:

   ```cpp
   std::vector<odom::TrackingWheelSource*> pods{};
   gOdometry.emplace(odom::kCherenkovTopology.kinematics, *gLeftIme, *gRightIme, *gImuSource, pods);
   ```

(Leaving unplugged pods configured also works — odometry treats them as broken
and falls back — but you'll get lower confidence readings.)

### One inertial sensor

In `main.cpp`, delete the `gSecondaryImu` lines and pass `nullptr` as the
second IMU:

```cpp
gImuSource.emplace(*gPrimaryImu, nullptr);
```

This also saves a few seconds of calibration at startup.

**Check:**

1. Push the robot forward exactly 24 inches. `y` should read about 24.
2. Spin it 360° in place. Heading should come back to the start, and `x` and
   `y` should stay close to where they started. **If `x`/`y` wander during a spin,
   a pod's offset sign is backwards.** Flip it and try again.

---

## Step 4 — Mechanisms

The demo arm (`subsystem::demo::ExampleArm`) is only an example. Replace it with
your real mechanisms using the pattern in
[subsystem § Writing a subsystem](layers/subsystem.md#writing-a-subsystem).

When you remove the demo arm, also update everything that uses it:

* `AutonomousContext::exampleArm` in `auton/autonomous_context.hpp`
* The demo button macro in `driver/demo_macros.cpp`
* The `"exampleArm.isExtended"` rule in `kDriveAccelLimitConfig`
  (`driver/driver_control_constants.hpp`)
* The dashboard, which shows the arm's state and fault box
  (`telemetry/dashboard.cpp`)
* The R1/R2/L1 bindings and status prints in `opcontrol()`

**Create every subsystem before `Scheduler::instance().start()`** in
`initialize()`, and before the `AccelLimitResolver` if its rules use the
subsystem's flags.

**Check:** press each button and watch the terminal print state changes.

---

## Step 5 — Driver feel

**File:** `include/lightspeed/driver/driver_control_constants.hpp`

These are up to your driver, not the programmer. Let them try each option:

* `kDriveMode` — `tank`, `arcade`, or `curvature`.
* `curveExponent` — higher gives finer control at low speed.
* `deadband` — just above where the sticks sit at rest.
* Acceleration limit rules — one per mechanism that should calm the drive when
  raised.

---

## Step 6 — Field and routines

* `auton/auton_constants.hpp` — check `kFieldDimensions` against the season's
  field. If the size changes, adjust `pixelsPerInch` so it still fits on screen.
* `auton/start_location.hpp` — replace "Placeholder Start A/B" with your real
  starting spots and headings.
* `auton/demo_routines.cpp` — replace the demo routines with real ones. See
  [auton § Writing a routine](layers/auton.md#writing-a-routine).

---

## Step 7 — Tune

Everything so far is measurement and wiring. Now follow the
[tuning guide](tuning.md) to set the control gains, in order.

---

## What you're getting

An honest summary.

**Solid:** the layer design, the odometry math, the "build everything once"
setup, safety features like clamped outputs and timeouts, and the logging.

**Placeholder:** almost every number. See the [placeholder checklist](placeholders.md).

**Untested:** everything. It has never run on a real robot. Treat the first
session as "does it work at all," not tuning.

**Known limitations:**

* `MoveToPose` and pure pursuit only drive **forward**. To back up, use
  `TurnToHeading` and `DriveStraightDistance`.
* `MoveToPose` and pure pursuit can stop short and wait out their timeout
  ([code review #4](code-review.md#4-movetopose-and-pure-pursuit-can-stop-short-then-wait-for-the-timeout)).
* Stall detection is too sensitive
  ([code review #2](code-review.md#2-stall-detection-trips-on-normal-loads)).
* Holonomic (mecanum/X) drivetrains aren't supported yet. The option exists but
  does nothing useful.
* Any subsystem can overwrite another's flag if they use the same name. Always
  prefix flag names with the subsystem name, like `"lift.isExtended"`.
* Fixed limits: 32 telemetry channels, 16 flags, 8 subsystems, 4 pods per role.
  Extra telemetry channels, flags, and subsystems are ignored. Extra pods are
  **not** checked.

See the full [code review](code-review.md) for everything else.
