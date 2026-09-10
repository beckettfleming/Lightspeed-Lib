---
title: Home
layout: home
nav_order: 1
permalink: /
description: Lightspeed — a from-scratch VEX V5RC robot library built on PROS.
---

**Lightspeed** is a from-scratch VEX V5RC robot code library written in C++ on top of
[PROS](https://pros.cs.purdue.edu/) 4.2.2. It was built for team **RoboPanthers (97934U)** and
covers everything a competition robot needs: motor and sensor wrappers, velocity control,
odometry, subsystems, driver control, autonomous motion, a touchscreen auton selector,
telemetry logging, and AprilTag vision correction.

It is not a wrapper around another library. Every layer — the PID controller, the odometry
fusion, the pure-pursuit follower, the GUI — is written in this repo.

---

## The one-paragraph version

Nine layers stack on top of each other. `hal` talks to hardware. `control` turns targets into
voltages. `odom` figures out where the robot is. `subsystem` runs mechanisms. `driver` handles
the joysticks. `motion` moves the robot to places. `auton` picks and runs routines. `telemetry`
records what happened. `vision` corrects the pose from AprilTags. Every one of them talks only
to the layer below it, and **every motion command in the entire codebase ends up at the same
`DrivetrainVelocityController::setTargetVelocity()` call** — driver control and autonomous share
one object, not two competing ones.

---

## Start here

| If you want to… | Read |
|---|---|
| Build and upload the code | **[Getting Started]({{ site.baseurl }}/getting-started/)** |
| Understand how the pieces fit together | **[Architecture]({{ site.baseurl }}/architecture/)** |
| Change a port number, gain, or field size | **[Configuration Reference]({{ site.baseurl }}/reference/configuration/)** |
| Wire up a brand-new robot | **[Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/)** |
| Tune the drivetrain and motion | **[Tuning Guide]({{ site.baseurl }}/guides/tuning/)** |
| Add your own mechanism | **[Subsystem Layer]({{ site.baseurl }}/layers/subsystem/)** |
| Write an autonomous routine | **[Autonomous Layer]({{ site.baseurl }}/layers/autonomous/)** |
| Fix something that's misbehaving | **[Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/)** |

---

## Layer map

```
                        ┌───────────────────────────┐
                        │   src/main.cpp            │  PROS entry points:
                        │   initialize / autonomous │  builds every shared object once
                        │   / opcontrol             │
                        └────────────┬──────────────┘
                                     │
        ┌──────────────┬─────────────┼─────────────┬──────────────┐
        │              │             │             │              │
   ┌────▼────┐   ┌─────▼─────┐  ┌────▼─────┐  ┌────▼─────┐  ┌─────▼──────┐
   │ driver  │   │   auton   │  │  motion  │  │telemetry │  │   vision   │
   │ (Step 5)│   │ (Step 7)  │  │ (Step 6) │  │ (Step 8) │  │  (Step 9)  │
   └────┬────┘   └─────┬─────┘  └────┬─────┘  └────┬─────┘  └─────┬──────┘
        │              │             │             │              │
        └──────────────┴──────┬──────┴─────────────┘              │
                              │                                   │
                    ┌─────────▼─────────┐                         │
                    │ subsystem (Step 4)│                         │
                    └─────────┬─────────┘                         │
                              │                                   │
              ┌───────────────┼───────────────┐                   │
              │               │               │                   │
        ┌─────▼─────┐   ┌─────▼─────┐         │                   │
        │  control  │   │   odom    │◄────────┼───────────────────┘
        │ (Step 2)  │   │ (Step 3)  │  applyVisionCorrection()
        └─────┬─────┘   └─────┬─────┘
              │               │
              └───────┬───────┘
                      │
                ┌─────▼─────┐
                │    hal    │   (Step 1) — the only layer that touches PROS device APIs
                └───────────┘
```

Each layer has its own wiki page:

- **[HAL Layer]({{ site.baseurl }}/layers/hal/)** — motors, IMU, rotation sensors, AI Vision, and the port map
- **[Control Layer]({{ site.baseurl }}/layers/control/)** — PIDF, slew limiting, drivetrain velocity control
- **[Odometry Layer]({{ site.baseurl }}/layers/odometry/)** — position tracking and sensor fusion
- **[Subsystem Layer]({{ site.baseurl }}/layers/subsystem/)** — the mechanism framework and scheduler
- **[Driver Control Layer]({{ site.baseurl }}/layers/driver-control/)** — joystick processing and the control map
- **[Motion Layer]({{ site.baseurl }}/layers/motion/)** — turn, drive, pure pursuit, move-to-pose
- **[Autonomous Layer]({{ site.baseurl }}/layers/autonomous/)** — routines, registry, and the touchscreen selector
- **[Telemetry Layer]({{ site.baseurl }}/layers/telemetry/)** — the data bus, SD logging, and the dashboard
- **[Vision Layer]({{ site.baseurl }}/layers/vision/)** — AprilTag pose correction
- **[Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/)** — the boot-time service mode

---

## Current status

The code **builds clean** with zero warnings under `-Wall -Wextra` and is architecturally
complete end to end. What it does **not** have yet is real robot data: port numbers, physical
measurements, control gains, and the season's field layout are all clearly-marked placeholders.

Nothing is stubbed out or faked — every feature is implemented and runnable. The placeholder
values just aren't *true* yet. See **[Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/)** for the complete list of what to
measure and where to put it.

> ⚠️ **Do not drive this on a real robot without working through [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/) first.**
> The placeholder gains and port numbers are guesses, and an untuned velocity controller on
> real hardware can behave violently.

---

## Conventions used everywhere

These hold across every layer, and knowing them up front makes the rest of the wiki obvious.

| Convention | Rule |
|---|---|
| **Naming** | `PascalCase` types, `camelCase` methods, `k`-prefixed constants |
| **Config** | Never a magic number in logic — every tunable lives in a `*_constants.hpp` file |
| **Ports** | Every raw port number lives in `include/lightspeed/hal/config.hpp`, nowhere else |
| **Heading** | Degrees, **clockwise-positive**, wrapped to `[0, 360)` |
| **Field frame** | At heading 0: robot forward → field **+y**, robot right → field **+x** |
| **Distance** | Inches everywhere above the HAL layer |
| **HAL units** | Raw and unconverted (motor degrees, sensor centidegrees) — conversion is a higher layer's job |
| **Voltage** | Millivolts, always clamped to ±12000 at the HAL boundary |
