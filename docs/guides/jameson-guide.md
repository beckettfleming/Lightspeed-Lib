---
title: Jameson Guide
parent: Guides
nav_order: 0
permalink: /guides/jameson-guide/
---

# The Jameson Guide

The minimum you need to know to add motors and sensors, make buttons do things in driver
control, and write an autonomous routine. You don't have to read anything else first.

You only ever touch **two files**:

| File | What goes in it |
|---|---|
| `include/hal/config.hpp` | Port numbers. **The only place a port number is allowed.** |
| `src/main.cpp` | Creating devices, driver control (`opcontrol`), and autonomous routines. |

The drivetrain, odometry, and motion code are already wired up. You don't need to touch them to
add a mechanism or write an auton.

---

## 1. Set up a motor

Adding a motor takes four small edits. This example adds a one-motor **lift** on port 14.

### a. Give it a port — `config.hpp`

Inside `namespace port { ... }`:

```cpp
inline constexpr std::int8_t kLift = 14;   // use -14 to reverse it
```

A **negative port number reverses the motor.** If it spins the wrong way, flip the sign here;
don't negate voltages in your code.

### b. Describe the group — `config.hpp`

Below the other `MotorGroupConfig`s:

```cpp
inline const MotorGroupConfig kLiftGroup{
    "liftGroup",
    {port::kLift},                  // more ports here = motors that always move together
    pros::v5::MotorGears::green,    // red = 100 RPM, green = 200, blue = 600
    pros::v5::MotorUnits::degrees,
};
```

Two motors on one mechanism that always spin together? Put both ports in the list. Two motors
that need to spin **opposite** ways? Make one port negative.

### c. Declare it — `main.cpp`

At the top of `main.cpp`, with the other `std::optional` globals:

```cpp
std::optional<lightspeed::hal::MotorGroup> gLift;
```

> **Why `std::optional`?** Devices can't be created before PROS starts. `std::optional` holds an
> empty slot until `initialize()` fills it. Use `gLift->` (arrow, not dot) to call it.

### d. Create it — `initialize()` in `main.cpp`

```cpp
gLift.emplace(hal::config::kLiftGroup);
gLift->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);   // optional: HOLD, BRAKE, or COAST
```

Use `HOLD` for anything that has to stay up against gravity (lifts, arms, winches). Use `COAST`
for intakes and flywheels.

### Making it move

There is one command: **`writeVoltage(millivolts)`**. Full power is `12000`, full reverse is
`-12000`, stop is `0`.

```cpp
gLift->writeVoltage(12000);    // up, full power
gLift->writeVoltage(-6000);    // down, half power
gLift->writeVoltage(0);        // stop (brake mode decides what "stop" feels like)
```

The motor **keeps doing the last thing you told it** until you tell it something else.

Useful reads:

| Call | Gives you |
|---|---|
| `gLift->getPositionRaw()` | Position in degrees (motor shaft, averaged across the group) |
| `gLift->getVelocityRpm()` | Speed in RPM |
| `gLift->getCurrentDrawMilliAmps()` | Current draw — good for "is it jammed / is it empty" checks |

---

## 2. Set up a sensor

### Pneumatics (and other 3-wire devices)

Pneumatic solenoids plug into the brain's **lettered** 3-wire ports (A–H).

**`config.hpp`**, inside `namespace adi_port { ... }`:

```cpp
inline constexpr std::uint8_t kMogoClamp = 'E';
```

**`main.cpp`** — declare, then create in `initialize()`:

```cpp
std::optional<pros::adi::DigitalOut> gMogoClamp;   // top of file
bool gMogoClampExtended = false;                   // remembers its state
```

```cpp
gMogoClamp.emplace(hal::config::adi_port::kMogoClamp);   // in initialize()
```

Then `gMogoClamp->set_value(true)` extends it and `set_value(false)` retracts it.

### Inertial sensor and tracking wheels

These are **already set up** — they feed odometry, which is how auton knows where the robot is.
You only need to set the right port numbers in `config.hpp`:

| Constant | Device |
|---|---|
| `kPrimaryImu`, `kSecondaryImu` | Inertial sensors. The second one is optional. |
| `kForwardPodRotation`, `kSidewaysPodRotation` | Rotation sensors on tracking wheels. Negative port = reversed. |

Missing sensors are fine — odometry notices and uses what's left. Just make sure a **different**
device isn't plugged into one of those ports.

> ⚠️ The inertial sensors calibrate when the program starts. **Don't touch the robot for a few
> seconds after you run the program**, or every turn in auton will be off. If you wired lights to
> `kCalibrationIndicator` / `kCalibratedIndicator`, the second one lights up when it's done.

---

## 3. Driver control (`opcontrol`)

`opcontrol()` in `main.cpp` is one big `while (true)` loop that runs every 10 ms. The driving code
is already in there. **You add your mechanism code inside that loop**, next to the existing
intake and winch code.

### The two button reads you need

| Call | True when… | Use it for |
|---|---|---|
| `master.get_digital(DIGITAL_R1)` | the button **is being held** | hold-to-run (intakes, lifts) |
| `master.get_digital_new_press(DIGITAL_R1)` | the button **was just pressed** (once per press) | toggles (pneumatics, modes) |

Button names: `DIGITAL_L1`, `L2`, `R1`, `R2`, `A`, `B`, `X`, `Y`, `UP`, `DOWN`, `LEFT`, `RIGHT`.

### Pattern: hold to run

```cpp
if (master.get_digital(DIGITAL_R1)) {
    gLift->writeVoltage(12000);
} else if (master.get_digital(DIGITAL_R2)) {
    gLift->writeVoltage(-12000);
} else {
    gLift->writeVoltage(0);
}
```

Always include the final `else` that stops the motor. Without it, the motor keeps running after
you let go.

### Pattern: toggle

```cpp
if (master.get_digital_new_press(DIGITAL_A)) {
    gMogoClampExtended = !gMogoClampExtended;
    gMogoClamp->set_value(gMogoClampExtended);
}
```

### Rules for the loop

- **Never `pros::delay()` or wait inside the loop** (except the one already at the bottom).
  While your code waits, the robot can't drive.
- **Never write a `while` loop that waits for a mechanism.** Check it once per pass instead —
  the loop will come back around in 10 ms.
- **Check the controller map before taking a button.** `Y` and `RIGHT` are pneumatics, `L1`/`L2`
  are the winch, `B`/`DOWN` are the intake. Search `main.cpp` for `DIGITAL_` to see what's taken.
- **Holding `Y` while the program starts** opens diagnostic mode instead of the normal program.

---

## 4. Autonomous

### How auton works

1. You write a **routine**: a function that calls motion commands one after another.
2. You **register** it in `initialize()` so it shows up on the brain screen.
3. Before the match, you tap the **start location**, then the **routine**, then **Confirm**.
4. When autonomous starts, your routine runs.

> You need a **competition switch** (or field control) to pick and run a routine. Without one,
> PROS skips straight to driver control.

### The motion commands

Every routine receives `ctx`, which holds all the motion commands. **Every one of them waits
until the move is finished** (or times out) before your next line runs.

| Command | What it does |
|---|---|
| `ctx.driveStraightDistance.run(24.0);` | Drive 24 in forward. Negative = backward. |
| `ctx.turnToHeading.run(90.0);` | Turn to **face** heading 90°. |
| `ctx.driveToPoint.turnToPoint(x, y);` | Turn to face a spot on the field. |
| `ctx.driveToPoint.driveToPoint(x, y);` | Turn to face a spot, then drive to it. |
| `ctx.moveToPose.run(x, y, heading);` | Curve to a spot and arrive facing `heading`. Forward only. |
| `ctx.purePursuit.follow({{x1, y1}, {x2, y2}, ...});` | Follow a smooth path through the points. Forward only. |
| `ctx.odometry.getPose()` | Where the robot thinks it is: `.xInches`, `.yInches`, `.headingDegrees`. |

### Units and directions — read this once

- **Distances are inches.** Coordinates are inches on the field.
- **Headings are degrees, clockwise.** 0° faces field +y, 90° faces +x, 180° faces −y, 270°
  faces −x.
- **`turnToHeading` is absolute, not relative.** `run(90.0)` means "face 90°", not "turn 90°
  more". Calling it twice in a row does nothing the second time.
- **`driveStraightDistance` is relative.** `run(24.0)` means "24 inches from where you are now".
- The robot's starting position comes from the **start location you tap** on the brain screen.
  Those are listed in `include/lightspeed/auton/start_location.hpp`. If you tap the wrong one,
  every `x, y` in your routine is wrong.

More detail: [Coordinate System]({{ site.baseurl }}/reference/coordinates/).

### Write a routine

Put it in `main.cpp`, **inside the `namespace { ... }` block, below the globals**, so it can use
your mechanisms (`gIntake`, `gLift`, ...):

```cpp
void runRedLeft(lightspeed::auton::AutonomousContext& ctx) {
    gIntake->writeVoltage(12000);          // start intake -- it keeps running...
    ctx.driveStraightDistance.run(24.0);   // ...while we drive 24 in
    gIntake->writeVoltage(0);              // stop intake

    ctx.turnToHeading.run(90.0);           // face +x
    ctx.driveToPoint.driveToPoint(48.0, 36.0);

    gMogoClamp->set_value(true);           // clamp
    pros::delay(250);                      // waiting IS fine in auton
    ctx.driveStraightDistance.run(-12.0);  // back up
}
```

Mechanism commands return instantly, so a mechanism you start **before** a motion command runs
**during** it. Unlike driver control, `pros::delay()` is fine here.

### Register it

In `initialize()`, next to the demo routines:

```cpp
gRoutineRegistry.registerRoutine(auton::Routine{
    .name = "Red Left",
    .validStartLocationIndices = {},   // {} = show for every start location
    .previewPoints = {},               // optional: {{x, y}, ...} draws the path on screen
    .run = runRedLeft,
});
```

Once you have real routines, delete the two `makeDemo...` lines so nobody picks a demo at a match.

### If auton misbehaves

| Problem | Likely cause |
|---|---|
| Nothing happens | No routine was tapped. Check the terminal for `no routine`. |
| Turns are way off | Robot moved during calibration, or wrong start location tapped. |
| Moves are slow, stop short, or wobble | The gains aren't tuned yet. See the [Tuning Guide]({{ site.baseurl }}/guides/tuning/). |
| Drives to the mirror-image spot | Mixed up +x / +y or clockwise. Re-read *Units and directions*. |
| Robot drives forward instead of backing to a point | `moveToPose` and `purePursuit` are forward-only. Use `driveStraightDistance` with a negative number. |

---

## Build and upload

```bash
pros mu    # build, upload, and open the terminal
```

Keep the terminal open while testing — the robot prints what it's doing there.

Next steps: [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/) for real hardware
values, [Tuning Guide]({{ site.baseurl }}/guides/tuning/) to make auton accurate.
