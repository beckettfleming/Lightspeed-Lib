# Lightspeed

<<<<<<< Updated upstream
📖 **Full documentation: <https://beckettfleming.github.io/Lightspeed-Lib/>**

New to the codebase? Start with
[Getting Started](https://beckettfleming.github.io/Lightspeed-Lib/getting-started/).
Setting up real hardware? Start with the
[Setup Checklist](https://beckettfleming.github.io/Lightspeed-Lib/guides/setup-checklist/).

---

A from-scratch VEX V5RC code library for team 97934U, built on PROS (C++). Layers: `lightspeed::hal` (Step 1),
`lightspeed::control` (Step 2), `lightspeed::odom` (Step 3),
`lightspeed::subsystem` (Step 4), `lightspeed::driver` (Step 5),
`lightspeed::motion` (Step 6), `lightspeed::auton` (Step 7),
`lightspeed::telemetry` (Step 8), `lightspeed::vision` (Step 9).
=======
Lightspeed is a VEX V5 robot code library written from scratch by team
**97934U**. It's built on [PROS](https://pros.cs.purdue.edu/) (C++).
>>>>>>> Stashed changes

It takes care of the parts of robot code that every team needs and that are hard
to get right:

* **Driving** — smooth, speed-controlled driving with joystick curves and
  acceleration limits.
* **Knowing where the robot is** — combines wheel encoders, inertial sensors, and
  tracking wheels into one field position.
* **Mechanisms** — a template for arms, lifts, and intakes with preset positions
  and automatic safety stops.
* **Autonomous** — drive a distance, turn to an angle, drive to a point, or follow
  a path, plus a touchscreen menu to pick the routine.
* **Logging** — records what the robot did to the SD card so you can look at it
  afterwards.
* **AprilTag vision** — an experimental position correction using the AI Vision
  sensor.

You set it up for your robot by changing **settings files**, not by rewriting the
library.

## Get started

| I want to… | Go to |
| --- | --- |
| Just get it working, no explanations | [**Easy mode**](docs/quick-start.md#easy-mode) |
| Get it working and understand the steps | [Quick start](docs/quick-start.md#quick-start-with-explanations) |
| Set it up properly on my own robot | [Porting guide](docs/porting.md) |
| Make it drive accurately | [Tuning guide](docs/tuning.md) |
| Learn the vocabulary | [Key ideas](docs/concepts.md) |
| See everything | [Documentation home](docs/README.md) |

## Status — read this first

* **It has never run on a real robot.** It was carefully written and reviewed,
  but that isn't the same as tested.
* **Almost every number is a placeholder.** Ports, wheel sizes, and control gains
  are sensible guesses waiting to be measured or tuned. They're all listed in the
  [placeholder checklist](docs/placeholders.md).
* **There are known issues.** A September 2026 review found some real problems,
  including one that caps driver speed at about half. See the
  [code review](docs/code-review.md). The quick start tells you how to work
  around the most important one.

## Build

```bash
pros build      # compile
pros upload     # compile and send to the brain
pros terminal   # watch messages from the brain (keep this open while testing)
```

Uses PROS kernel 4.2.2 and LVGL 9.2.0.

## How it's organized

The library is split into ten parts. Each one is a folder under
`include/lightspeed/` and `src/lightspeed/`.

| Part | What it does |
| --- | --- |
| [`hal`](docs/layers/hal.md) | Port numbers and simple wrappers around motors and sensors |
| [`control`](docs/layers/control.md) | Speed control for the drivetrain (PID, ramping) |
| [`odom`](docs/layers/odom.md) | Tracks the robot's position on the field |
| [`subsystem`](docs/layers/subsystem.md) | Template for mechanisms like arms and intakes |
| [`driver`](docs/layers/driver.md) | Turns joystick input into drive commands |
| [`motion`](docs/layers/motion.md) | Autonomous movements: drive, turn, go to point, follow path |
| [`auton`](docs/layers/auton.md) | Autonomous routines and the touchscreen picker |
| [`telemetry`](docs/layers/telemetry.md) | Logging, brain-screen dashboard, live data over USB |
| [`vision`](docs/layers/vision.md) | AprilTag position correction (experimental, off by default) |
| [`diagnostics`](docs/layers/diagnostics.md) | A test mode you enter by holding Y at startup |

## Ground rules the code follows

* **One path to the drive motors.** Driver control and autonomous both send
  speeds through the same drivetrain controller.
* **One of everything.** Each motor group, controller, and sensor object is
  created once in `src/main.cpp` and shared.
* **One file for ports.** Only `include/lightspeed/hal/config.hpp` has port
  numbers.
* **No magic numbers in the logic.** Every setting is in a `*_constants.hpp`
  file.
