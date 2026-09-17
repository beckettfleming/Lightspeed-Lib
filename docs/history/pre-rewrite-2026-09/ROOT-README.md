# Lightspeed

A from-scratch VEX V5RC code library for team **97934U**, built on PROS (C++).

Lightspeed is a full framework rather than a set of helpers. It covers hardware
abstraction, closed-loop control, sensor-fused odometry, a subsystem framework,
driver control, autonomous motion planning, an on-brain touch auton selector,
telemetry and logging, and AprilTag pose correction — ten layers, each one
config-driven so the same library can move from one robot to the next by editing
constants rather than logic.

```
lightspeed::hal          port map, motor groups, IMU, rotation + AI vision sensors
lightspeed::control      PIDF, slew limiting, drivetrain velocity control
lightspeed::odom         IME / IMU / 0-4 tracking pods → fused field pose @200Hz
lightspeed::subsystem    subsystem base class, shared scheduler, named flag registry
lightspeed::driver       input profiling, drive modes, accel limiting, button macros
lightspeed::motion       motion profiles, turn/drive/point/pose, pure pursuit
lightspeed::auton        routine registry, start locations, two-screen touch selector
lightspeed::telemetry    telemetry bus, SD CSV logging, brain dashboard, serial link
lightspeed::vision       AprilTag solving, trust gating, odometry correction
lightspeed::diagnostics  boot-gated service mode
```

## Documentation

**→ [`docs/`](docs/README.md)** — architecture, per-layer reference, tuning
guide, porting guide.

| | |
| --- | --- |
| How it fits together | [Architecture](docs/architecture.md) |
| Build, upload, drive | [Getting started](docs/getting-started.md) |
| Put it on another robot | [Porting](docs/getting-started.md#porting-lightspeed-to-a-different-robot) |
| Make it drive well | [Tuning guide](docs/tuning.md) |
| What is still a placeholder | [Bucket B checklist](docs/BUCKET_B_CHECKLIST.md) |

## Design principles

**One path to the motors.** Driver control, every motion primitive, and every
autonomous routine all end at
`control::DrivetrainVelocityController::setTargetVelocity()`. Nothing outside
`hal::MotorGroup` writes voltage to the drive.

**One instance of everything.** `main.cpp` constructs each hardware and control
object once in `initialize()` and hands out references. Autonomous and driver
control share the same drivetrain controller, the same odometry, the same motion
primitives — not two copies fighting over the same ports.

**One place for every port number.** `hal/config.hpp` is the only file in the
project containing a raw port. Rewiring the robot is a one-file edit.

**Constants never live in logic.** Every tunable is in a `*_constants.hpp` file
or a config struct passed at construction.

**Topology is configuration.** The odometry core takes 0–4 tracking pods in any
mix of roles, degrades through a confidence hierarchy as sensors fail, and
reports how much it trusts itself. It is not hardcoded to one pod layout,
because the next robot's will be different.

**Voltage-based control.** Custom feedforward plus PID via `move_voltage`,
rather than PROS's built-in `move_velocity`.

## Status

The project compiles clean under `-Wall -Wextra` on ARM GNU Toolchain 14.2.1,
zero warnings, and its cross-layer wiring and fail-safe paths have been audited
line by line.

It has **never been run on physical hardware**, and nearly every numeric constant
in it is a clearly-marked placeholder waiting on a measurement, a tuning session,
or season data. Nothing was guessed at to look finished — every placeholder is
tracked in [`docs/BUCKET_B_CHECKLIST.md`](docs/BUCKET_B_CHECKLIST.md) with a
pointer to exactly where it lives.

The full review-pass report that used to live here is preserved at
[`docs/history/review-pass.md`](docs/history/review-pass.md).

## Building

```bash
pros build      # PROS kernel 4.2.2, liblvgl 9.2.0, target v5
pros upload
pros terminal   # the library prints a lot — keep this open while bench testing
```
