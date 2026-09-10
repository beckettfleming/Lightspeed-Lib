---
title: Architecture
nav_order: 3
permalink: /architecture/
---

This page explains the shape of the codebase: how the layers relate, what runs on which
background task, and the design rules that hold everywhere.

---

## The layering rule

Nine layers, each depending only on layers below it:

| # | Namespace | Job | Depends on |
|---|---|---|---|
| 1 | `lightspeed::hal` | Talk to physical devices | PROS only |
| 2 | `lightspeed::control` | Turn targets into voltages | `hal` |
| 3 | `lightspeed::odom` | Track where the robot is | `hal` |
| 4 | `lightspeed::subsystem` | Run mechanisms with state machines | `hal`, `control` |
| 5 | `lightspeed::driver` | Process joystick input | `control`, `subsystem` |
| 6 | `lightspeed::motion` | Move the robot to a place | `control`, `odom` |
| 7 | `lightspeed::auton` | Pick and run routines | `motion`, `subsystem`, `odom` |
| 8 | `lightspeed::telemetry` | Record and display state | everything (passively) |
| 9 | `lightspeed::vision` | Correct the pose from AprilTags | `hal`, `odom` |
| — | `lightspeed::diagnostics` | Boot-time service mode | `telemetry`, `vision` |

**`hal` is the only layer that touches PROS device APIs.** If you find yourself writing
`pros::Motor` outside `include/lightspeed/hal/`, stop — add a HAL wrapper instead.

---

## The single-output-path rule

This is the most important structural decision in the project.

```
  Driver joysticks                Autonomous routine
        |                                |
        v                                v
  input profiling                 TurnToHeading
        |                         DriveStraightDistance
        v                         DriveToPoint
  drive-mode transform            MoveToPose
        |                         PurePursuitController
        v                                |
  accel-limited slew                     |
        |                                |
        +----------------+---------------+
                         v
      DrivetrainVelocityController::setTargetVelocity(leftRpm, rightRpm)
                         |
                         v
        100Hz control task: PIDF + feedforward + slew + battery comp
                         |
                         v
            hal::MotorGroup::writeVoltage()   <-- the only place voltage is written
```

**No motion code anywhere writes voltage directly.** Every primitive ends by calling
`setTargetVelocity()`. This has been verified by grepping for `move_voltage` / `writeVoltage`
across the whole motion layer — zero hits outside `hal::MotorGroup` itself.

Why it matters: the drivetrain's feedforward, PID trim, slew limiting, battery-sag
compensation, and motor-health fail-safes apply *equally* to driver control and autonomous.
There is no second, less-safe path.

---

## The shared-globals rule

`src/main.cpp` builds every long-lived object exactly once, in `initialize()`, as
file-scope `std::optional<T>` globals:

```cpp
std::optional<lightspeed::hal::MotorGroup> gLeftDrive;
std::optional<lightspeed::control::DrivetrainVelocityController> gDrivetrain;
std::optional<lightspeed::odom::OdometryFusion> gOdometry;
// ...and so on
```

`autonomous()` and `opcontrol()` both reference **the same instances**. This is deliberate, not
laziness:

- The auton selector GUI must be usable in `initialize()`, before either period runs — so
  odometry and the motion primitives have to already exist at that point.
- If `opcontrol()` built its own second `DrivetrainVelocityController` on the same ports, two
  independent PIDF loops would fight over the same physical motors.

`std::optional` is used rather than plain globals because these objects need PROS to be
initialized before their constructors run. `emplace()` inside `initialize()` gives explicit
control over *when* construction happens.

---

## Background tasks

Lightspeed runs several PROS tasks concurrently. Knowing what they are prevents a lot of
confusion.

| Task | Rate | Owned by | What it does |
|---|---|---|---|
| `lightspeed_odometry_fusion` | **200 Hz** (5 ms) | `OdometryFusion` | Reads sensors, integrates pose |
| Drivetrain control loop | **100 Hz** (10 ms) | `DrivetrainVelocityController` | PIDF + slew + battery comp to voltage |
| Subsystem scheduler | **50 Hz** (20 ms) | `Scheduler` | Calls `update()` on every subsystem |
| `opcontrol()` loop | **100 Hz** (10 ms) | PROS | Reads joysticks, sets targets |
| Auton selector GUI | on demand | `SelectorGui` | Polls touches, redraws |
| Dashboard | **8 Hz** (125 ms) | `Dashboard` | Draws the driver-control screen |
| SD logger | **25 Hz** sample | `SdLogger` | Buffers CSV rows, flushes every 10 |
| Serial link | **10 Hz** | `SerialLink` | Streams CSV over USB (diagnostic mode only) |

Full detail on **[Task and Timing Reference]({{ site.baseurl }}/reference/tasks/)**.

### Who owns the screen

Only one task may draw to `pros::screen` at a time.

```
  initialize()   ->  SelectorGui owns the screen
  autonomous()   ->  GUI stopped; nothing draws
  opcontrol()    ->  GUI stopped; Dashboard draws
```

`Dashboard` enforces this itself — it only draws when
`!is_disabled() && !is_autonomous()`, i.e. only during driver control. Both `autonomous()` and
`opcontrol()` call `gSelectorGui->stop()` at their top (the `opcontrol()` call is idempotent
insurance for bench testing where `autonomous()` never runs).

### Thread safety

Anything crossing a task boundary is guarded:

- `OdometryFusion` — a `pros::MutexVar<FusedState>` for pose data, plus a separate `resetMutex_`
  held for a whole fusion cycle so `setPose()` and `applyVisionCorrection()` are atomic against it.
- `DrivetrainVelocityController` — `std::atomic<double>` targets, so `setTargetVelocity()` is
  safe from any task with no lock at all.
- `Subsystem<StateEnum>` — a recursive mutex around the whole `update()` cycle and every state
  transition, so an external `moveToPreset()` can't land mid-cycle.
- `TelemetryBus` and `FlagRegistry` — a short-lived mutex around a fixed-size array scan.

---

## Config-over-magic-numbers

Every layer that has tunable values puts them in a dedicated `*_constants.hpp` file, and the
class itself never hardcodes one:

| File | Holds |
|---|---|
| `hal/config.hpp` | Every port number and motor-group definition |
| `control/drivetrain_velocity_constants.hpp` | Drivetrain PIDF gains, slew rate, battery thresholds |
| `odom/odometry_constants.hpp` | Pod topology, wheel diameter, gear ratio |
| `driver/driver_control_constants.hpp` | Drive mode, input curve, max RPM, accel-limit rules |
| `motion/motion_constants.hpp` | Track width and every motion primitive's gains and limits |
| `auton/auton_constants.hpp` | Field dimensions and the field-to-screen mapping |
| `auton/screen_layout.hpp` | Every pixel coordinate in the selector GUI |
| `telemetry/telemetry_constants.hpp` | Every telemetry rate and buffer size |
| `telemetry/dashboard_layout.hpp` | Dashboard line positions |
| `vision/vision_constants.hpp` | Camera calibration, mount offset, tag map, gating |

The practical payoff: **rewiring the robot means editing one file; retuning means editing one
file per layer.** See **[Configuration Reference]({{ site.baseurl }}/reference/configuration/)** for the full annotated list.

---

## Fail-safe behavior

Safety checks live in the layers rather than being sprinkled at call sites:

| Condition | What happens |
|---|---|
| Drive motor **stalled** or **over temperature** | Output zeroed, PIDF integrator and slew limiter reset |
| Drive motors **all disconnected** | Output zeroed |
| Drive motors **partially disconnected** | Keeps driving in a degraded state — a side with one working motor is still drivable |
| **Battery below 11 V** | Sag compensation stops boosting further (capped at 1.0x, never cut below — a low pack still needs to drive off the field) |
| Subsystem motor faulted | `Subsystem::driveToPosition()` zeroes voltage *unconditionally*, whether or not the concrete subsystem's `onFault()` reacts |
| Motion primitive doesn't settle | Every blocking primitive has both a settle condition **and** a timeout |
| SD card pulled mid-match | `fwrite` return value checked; logging stops cleanly instead of retrying a dead file |
| No routine confirmed before auton | Falls back to the last previewed routine, and logs that it did |
| Any voltage write | Clamped to +/-12000 mV at the HAL boundary |

---

## How telemetry gets published

Telemetry is push-based and passive: layers call `TelemetryBus::record(name, value)` from
their own existing update cycles. The bus never polls anything, and consumers (`SdLogger`,
`Dashboard`) read snapshots at their own independent rate.

That means adding a value to the log is a one-line change in the layer that owns it — no
plumbing. See **[Telemetry Layer]({{ site.baseurl }}/layers/telemetry/)** for the full channel list.
