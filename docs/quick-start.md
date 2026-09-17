# Quick start

This page has two parts:

* **[Easy mode](#easy-mode)** — just the steps to get a robot driving and
  running autonomous. No explanations.
* **[Quick start](#quick-start-with-explanations)** — the same path with short
  explanations of what's happening and what to look for.

---

## Easy mode

Follow these in order. Don't skip the ⚠️ steps.

### What you need

- [ ] A computer with **VS Code** and the **PROS extension** installed
- [ ] A **V5 Brain**, a **V5 Controller**, and a **USB cable**
- [ ] A **tank-style drivetrain** with 3 motors per side
- [ ] Wheels **off the ground** (robot on a stand or blocks) for the first test

### 1. Open the project

- [ ] Open the `Lightspeed Lib` folder in VS Code.

### 2. Set your motor ports

Open `include/lightspeed/hal/config.hpp`.

- [ ] Set these six numbers to the ports your drive motors are plugged into:

  ```cpp
  inline constexpr std::int8_t kLeftDriveFront = 1;
  inline constexpr std::int8_t kLeftDriveMiddle = 2;
  inline constexpr std::int8_t kLeftDriveBack = 3;
  inline constexpr std::int8_t kRightDriveFront = -4;
  inline constexpr std::int8_t kRightDriveMiddle = -5;
  inline constexpr std::int8_t kRightDriveBack = -6;
  ```

- [ ] Put a **minus sign** in front of any motor that spins the wrong way.
  (Usually the whole right side is negative.)
- [ ] Set `kPrimaryImu` to the port of your inertial sensor.
- [ ] Make sure **nothing else** is plugged into ports 7–13, or change those
  numbers to empty ports. Sensors you don't have are fine — the code works
  without them.

### 3. Set your motor color

Still in `config.hpp`:

- [ ] In both `kLeftDriveGroup` and `kRightDriveGroup`, set the cartridge:
  `pros::v5::MotorGears::blue` (600 RPM), `green` (200 RPM), or `red` (100 RPM).

### 4. Set your top speed ⚠️

- [ ] In `include/lightspeed/driver/driver_control_constants.hpp`, set
  `kMaxDriveRpm` to your cartridge's RPM: **600** blue, **200** green, **100** red.
- [ ] In `include/lightspeed/control/drivetrain_velocity_constants.hpp`, set
  `.kV` to **12000 ÷ that same number** (blue = `20.0`, green = `60.0`, red = `120.0`).

### 5. Set your wheels and gearing

Change these in **both** files:
`include/lightspeed/motion/motion_constants.hpp` (`kCherenkovKinematics`) and
`include/lightspeed/odom/odometry_constants.hpp` (`kDriveImeConfig`).

- [ ] `wheelDiameterInches` — your drive wheel size (for example `3.25` or `4.0`)
- [ ] `gearRatio` — **wheel RPM ÷ motor RPM**. Direct drive = `1.0`.
  Blue motors geared to 450 RPM = `450.0 / 600.0`.

In `motion_constants.hpp` only:

- [ ] `trackWidthInches` — distance from the middle of the left wheels to the
  middle of the right wheels.

### 6. Build and upload

- [ ] Plug the brain into your computer with USB.
- [ ] In VS Code, open a terminal and run:

  ```bash
  pros mu
  ```

  (Or use the PROS extension's **Upload** button, then **Brain Terminal**.)

### 7. Start the program ⚠️

- [ ] Run the program on the brain.
- [ ] **Don't touch or move the robot for about 6 seconds** while the sensors
  calibrate.

### 8. Drive

With no competition switch plugged in, driver control starts on its own.

| Control | Does |
| --- | --- |
| Left stick up/down | Forward / backward |
| Right stick left/right | Turn |
| R1 / R2 | Demo arm up / down (port 12, if you have a motor there) |
| L1 | Demo arm up-then-down |

- [ ] **Robot spins instead of driving straight?** Flip the minus signs on one
  side (step 2).
- [ ] **Forward is backward?** Flip the minus signs on **all six** motors.
- [ ] Once it drives the right way on blocks, put it on the ground.

### 9. Run an autonomous routine ⚠️

You need a **competition switch** (or field controller) for this.

- [ ] Plug the competition switch into the controller. Set it to **Disabled**.
- [ ] Start the program. Wait for calibration.
- [ ] **Put the robot where it will start.** Clear 3 feet of space in front of it.
- [ ] On the brain screen, tap a **start location** (yellow dot).
- [ ] Tap a **routine** in the list.
- [ ] Tap **Confirm**. It turns green.
- [ ] Flip the switch to **Autonomous** and **Enabled**.

The demo routines are slow and may stop early — the gains aren't tuned yet.
That's expected. Next step: the [tuning guide](tuning.md).

### If something is wrong

| Problem | Try |
| --- | --- |
| Won't build | Check for a missing `;` or `,` near what you edited. |
| Robot doesn't move at all | Check motor ports. Check the terminal for `DISCONNECTED`. |
| One side doesn't move | That side's ports are wrong, or all its motors are unplugged. |
| Drive pulses on and off when pushing | Known issue: stall detection is too sensitive. See [code review #2](code-review.md#2-stall-detection-trips-on-normal-loads). |
| Robot is slow at full stick | Redo step 4. |
| Selector screen vanishes right away | No competition switch — that's normal. See step 9. |
| Autonomous does nothing | You didn't tap a routine. Check the terminal for `no routine`. |
| Robot drifts after being placed | Place the robot **before** tapping the start location. |

---

## Quick start (with explanations)

This is the same path as easy mode, with the reasons behind each step.

### What Lightspeed is

Lightspeed is a code library for VEX V5 robots, written in C++ using
[PROS](https://pros.cs.purdue.edu/). It handles the hard, reusable parts of
robot code: driving smoothly, knowing where the robot is on the field, running
mechanisms, choosing and running autonomous routines, and logging data.

You mostly change **numbers in settings files**, not the library's logic.

### Before you start

* **Install PROS.** The project uses PROS kernel 4.2.2 and LVGL 9.2.0 (listed in
  `project.pros`). The PROS VS Code extension installs everything you need.
* **Know the three PROS entry points.** Every PROS program has:
  * `initialize()` — runs once when the program starts.
  * `autonomous()` — runs during the 15-second autonomous period.
  * `opcontrol()` — runs during driver control.

  All three are in `src/main.cpp`.

### Where settings live

Every setting is in a file ending in `_constants.hpp`, plus the port map in
`hal/config.hpp`. See [Where things live](architecture.md#where-things-live)
for the full list.

Almost every number in those files is a **placeholder**. It was a reasonable
guess, not a measurement. The [placeholder checklist](placeholders.md) lists
them all.

### Step 1 — Wire up the ports

`include/lightspeed/hal/config.hpp` is the **only** file with port numbers in it.

* A **negative** port number reverses that motor. Motors on the right side of a
  tank drive usually need to be reversed so that "positive" means "forward" on
  both sides.
* The default setup expects 3 tracking wheels (ports 7–9), 2 inertial sensors
  (10–11), a demo arm motor (12), and an AI Vision sensor (13).
* **Missing sensors are OK.** If a tracking wheel or the second IMU isn't
  plugged in, odometry notices and uses what's left. Just make sure a
  *different* device isn't plugged into one of those ports.

### Step 2 — Make the speed numbers agree

The drivetrain controller works in **motor RPM** — the speed of the motor's
own output shaft (up to 600 for blue). Everything that sends it a target has
to use the same unit.

The defaults assumed **wheel** RPM, which is a mistake (see
[code review #1](code-review.md#1-drivetrain-speed-units-disagree--driver-control-is-capped-at-57-speed)).
So set `kMaxDriveRpm` to the cartridge RPM, and set `kV` to 12000 ÷ that.
`kV` is the "feedforward": how many millivolts to send per RPM of target speed,
before any correction. 12000 mV is full power.

Then set wheel size, gear ratio, and track width. These let the library turn
"drive 24 inches" into motor speeds, and turn encoder counts back into inches.

### Step 3 — Build, upload, and watch the terminal

```bash
pros build      # compile only
pros upload     # compile and send to the brain
pros terminal   # show messages from the brain
pros mu         # upload, then open the terminal
```

**Keep the terminal open.** Lightspeed prints a lot of useful information:

* `[opcontrol] target=... slewed=...` once a second — proof driver control is
  running, and the speeds it's asking for.
* `[opcontrol] accel limit changed` — when the arm goes up or down, the drive
  gets gentler on purpose.
* `[ExampleArm] entering HOLDING` — demo arm state changes.
* `[autonomous] running confirmed routine: ...` and the final position.
* `[SdLogger] logging to '/usd/lslog003.csv'` — if an SD card is in.

### Step 4 — What happens at startup

`initialize()` does the following, in order:

1. Creates the motor groups and starts the drivetrain speed controller.
2. **Calibrates both inertial sensors, one after the other.** This takes a few
   seconds each and the robot must stay still.
3. Starts odometry (position tracking), the motion commands, and the demo arm.
4. Registers the demo autonomous routines.
5. Checks if **Y** is being held on the controller. If so, it switches to
   [diagnostic mode](layers/diagnostics.md) and stops here.
6. Shows the autonomous selector on the brain screen, and starts the SD card
   logger and the driver dashboard.

### Step 5 — Driver control

Every 20 ms, `opcontrol()`:

1. Reads the joysticks.
2. Applies a **deadband** (ignores tiny stick movement) and a **curve** (makes
   small stick movements gentler for precise control).
3. Turns the sticks into left and right speeds. The default is **arcade**
   (left stick = forward, right stick = turn). Tank and curvature are also
   available.
4. Limits how fast the speed can change (**acceleration limit**). This limit
   gets stricter while the arm is raised, so the robot doesn't tip.
5. Sends the result to the drivetrain speed controller.

The brain screen shows a dashboard with battery, position, and arm state.

### Step 6 — Autonomous

The brain screen selector needs the robot to sit in the **disabled** state,
which only happens with a competition switch or field control. Without one,
PROS goes straight to driver control and the selector closes.

With a switch:

1. **Screen 1:** tap where the robot is starting. This tells odometry where the
   robot is. Do this *after* placing the robot.
2. **Screen 2:** tap a routine to preview its path, then tap **Confirm**.
3. When autonomous starts, the chosen routine runs.

If you tapped a routine but forgot **Confirm**, it runs that routine anyway and
prints a warning. Doing nothing for 15 seconds is worse.

> **No competition switch?** In the current code there is no other way to pick
> and run a routine. Borrow a switch (they're inexpensive), or ask a programmer
> to add a temporary test button.

### Step 7 — Logs

Put a microSD card in the brain. Every time the robot goes from disabled to
enabled, the logger starts a new file (`/usd/lslog000.csv`, `lslog001.csv`, …)
with position, speeds, and health about 25 times per second. Open it in a
spreadsheet or plotting tool.

### Where to go next

| Goal | Read |
| --- | --- |
| Put Lightspeed on a different robot properly | [Porting guide](porting.md) |
| Make it drive accurately | [Tuning guide](tuning.md) |
| Understand how the pieces connect | [Architecture](architecture.md) |
| Learn the vocabulary (odometry, PID, pose…) | [Key ideas](concepts.md) |
| Know what's broken or risky | [Code review](code-review.md) |
