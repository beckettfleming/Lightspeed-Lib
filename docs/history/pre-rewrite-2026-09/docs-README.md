# Lightspeed documentation

Lightspeed is a from-scratch VEX V5RC code library built on PROS (C++20), written
and maintained by team **97934U**. It is a full framework rather than a
collection of helpers: hardware abstraction, closed-loop control, odometry,
subsystems, driver control, autonomous motion, an on-brain auton selector,
telemetry, and AprilTag pose correction.

These docs are written for anyone on **97934** — the team that maintains it and
any sibling team adopting it on a different robot. They assume you know PROS
basics (`initialize()` / `autonomous()` / `opcontrol()`, `pros::Task`, building
and uploading), but assume nothing about this particular robot.

## Start here

| If you want to… | Read |
| --- | --- |
| Understand how the library is put together | [Architecture](architecture.md) |
| Build it, upload it, and drive | [Getting started](getting-started.md) |
| Put it on a different robot | [Getting started § Porting](getting-started.md#porting-lightspeed-to-a-different-robot) |
| Look up a specific class or config struct | [Layer reference](#layer-reference) |
| Tune the gains so it actually drives well | [Tuning guide](tuning.md) |
| Know what is still a placeholder | [Bucket B checklist](BUCKET_B_CHECKLIST.md) |

## Layer reference

Layers are listed bottom-up. Each one only depends on the layers above it in
this list, never below — see [Architecture](architecture.md#dependency-rules).

| Namespace | Page | What it owns |
| --- | --- | --- |
| `lightspeed::hal` | [hal](layers/hal.md) | Port map, motor groups, IMU, rotation sensors, AI Vision sensor |
| `lightspeed::control` | [control](layers/control.md) | PIDF, slew limiting, drivetrain velocity control |
| `lightspeed::odom` | [odom](layers/odom.md) | Pose tracking: IME, IMU, tracking pods, sensor fusion |
| `lightspeed::subsystem` | [subsystem](layers/subsystem.md) | Subsystem base class, scheduler, named flag registry |
| `lightspeed::driver` | [driver](layers/driver.md) | Input profiling, drive modes, accel limiting, button macros |
| `lightspeed::motion` | [motion](layers/motion.md) | Motion profiles, turn/drive/point/pose primitives, pure pursuit |
| `lightspeed::auton` | [auton](layers/auton.md) | Routine registry, start locations, two-screen touch selector |
| `lightspeed::telemetry` | [telemetry](layers/telemetry.md) | Telemetry bus, SD logging, brain dashboard, serial link |
| `lightspeed::vision` | [vision](layers/vision.md) | AprilTag solving, gating, odometry correction |
| `lightspeed::diagnostics` | [diagnostics](layers/diagnostics.md) | Boot-gated service mode |

## A note on honesty

This library compiles clean and is structurally complete, but a large number of
its constants are **placeholders** waiting on physical measurements, tuning, or
season data that did not exist when it was written. Every one of them is tracked
in [`BUCKET_B_CHECKLIST.md`](BUCKET_B_CHECKLIST.md) and flagged in these docs
with a ⚠️ marker. Do not read a number in this codebase as a tuned value unless
the checklist says it has been confirmed.

The [full review-pass report](history/review-pass.md) that used to be the
project README is kept for provenance.
