# Lightspeed

Robot code for VEX V5RC team **97934U Tachyon**, written in C++ on
[PROS](https://pros.cs.purdue.edu/) 4.2.2. All of it, from the PID loops to the auton
selector, is written in this repo.

📖 **Docs:** <https://beckettfleming.github.io/Lightspeed-Lib/>
New here? Read the [Jameson Guide](docs/guides/jameson-guide.md) first.

---

## What the program does

Everything starts in [`src/main.cpp`](src/main.cpp), which holds the three PROS entry points.

### `initialize()`: when the program starts

1. Creates every motor group, sensor, and pneumatic **once**. Driver control and auton share the
   same objects.
2. Calibrates the inertial sensors. **Don't move the robot during this.** The ADI indicator on
   port B is lit while it calibrates, and the one on port A lights when it's done.
3. Starts odometry, which tracks the robot's field position from the drive encoders, tracking
   wheels, and IMUs.
4. Registers the autonomous routines.
5. If **Y is held** at startup, enters diagnostic mode and stops there.
6. Otherwise, shows the auton selector on the brain screen and starts the SD-card logger and the
   driver dashboard.

### `autonomous()`

Runs the routine picked on the brain screen. On the selector you tap the start location, then
the routine, then **Confirm**. If a routine was picked but never confirmed, it runs that routine
anyway and prints a warning. Selecting a routine needs a competition switch or field control.

Routines are written from a small set of blocking motion commands:

| Command | Does |
|---|---|
| `driveStraightDistance.run(in)` | Drive a distance forward/back, holding heading |
| `turnToHeading.run(deg)` | Turn in place to an absolute field heading |
| `driveToPoint.driveToPoint(x, y)` | Turn to face a point, then drive to it |
| `moveToPose.run(x, y, deg)` | Curve to a point and arrive at a heading |
| `purePursuit.follow({...})` | Follow a smoothed path through waypoints |

### `opcontrol()`: driver control

A 10 ms loop. Joystick input passes through a deadband and curve, then a drive mode (arcade by
default), then an acceleration limit. The result goes to the same drivetrain velocity
controller that auton uses.

| Control | Does |
|---|---|
| Sticks | Drive |
| **L1** (hold) | Wind winch. Stops on its own when current drops, meaning the winch is unloaded |
| **L2** (hold) | Let winch out. Stops at its starting position |
| **B** / **DOWN** (hold) | Intake in / out |
| **Y** | Toggle claw pneumatic |
| **RIGHT** | Toggle hinge pneumatic |

---

## Build

```bash
pros build      # compile
pros mu         # compile, upload, and open the terminal
```

Keep `pros terminal` open while testing. The robot prints its state and auton progress there.

---

## Where things are

| You want to change… | File |
|---|---|
| Port numbers (the only place they appear) | [`include/lightspeed/hal/config.hpp`](include/lightspeed/hal/config.hpp) |
| Buttons, mechanisms, auton routines | [`src/main.cpp`](src/main.cpp) |
| Gains, speeds, wheel sizes | `include/lightspeed/*/*_constants.hpp` |
| Auton start locations | [`include/lightspeed/auton/start_location.hpp`](include/lightspeed/auton/start_location.hpp) |

The library is split into layers under `include/lightspeed/` and `src/lightspeed/`. Each one
only uses the layers below it:

| Layer | Job |
|---|---|
| `hal` | Port map and thin motor/sensor wrappers |
| `control` | PIDF, slew limiting, drivetrain velocity control |
| `odom` | Fuses encoders, tracking wheels, and IMUs into a field pose |
| `subsystem` | Framework for mechanisms with states and safety stops |
| `driver` | Joystick curves, drive modes, accel limits, button macros |
| `motion` | The auton motion commands above |
| `auton` | Routine registry and touchscreen selector |
| `telemetry` | SD logging, brain dashboard, USB serial data |
| `vision` | AprilTag pose correction (experimental) |
| `diagnostics` | Hold-Y test mode |

Many gains and measurements are still placeholders. See the
[Setup Checklist](docs/guides/setup-checklist.md) and [Tuning Guide](docs/guides/tuning.md).
