# `lightspeed::driver` — driver control

This layer turns joystick input into wheel speeds. It's everything between the
controller and `DrivetrainVelocityController::setTargetVelocity()`.

Every 20 ms, `opcontrol()` runs these steps:

```
joysticks
   → InputProfile          (deadband + curve)
   → computeDriveOutput    (tank / arcade / curvature → left & right, −1 to 1)
   → × kMaxDriveRpm        (turn into motor RPM)
   → SlewRateLimiter       (acceleration limit, set by AccelLimitResolver)
   → setTargetVelocity()   (the same call autonomous uses)
```

Each step is its own small piece you can change or test separately.

---

## `InputProfile` — deadband and curve

Cleans up one joystick axis.

```cpp
struct InputProfileConfig {
    double curveExponent = 1.0;   // 1 = straight line; higher = gentler at low stick
    double deadband = 0.0;        // ignore stick movement smaller than this (0 to 1)
};

explicit InputProfile(const InputProfileConfig& config);

double apply(double rawInput) const;   // input and output are −1 to 1
void   setConfig(const InputProfileConfig& config);
const InputProfileConfig& getConfig() const;
```

What `apply()` does:

1. Limits the input to −1…1.
2. If it's inside the deadband, returns 0.
3. Stretches the rest so output starts at 0 right at the edge of the deadband
   (no sudden jump).
4. Raises it to `curveExponent` and keeps the sign.

Full stick is still full speed, whatever the exponent. `opcontrol()` uses one
profile for all four stick axes.

---

## Drive modes

```cpp
enum class DriveMode { tank, arcade, curvature };

struct JoystickInput { double leftY, leftX, rightY, rightX; };
struct DriveOutput   { double left, right; };   // −1 to 1

DriveOutput tankDrive(double leftYInput, double rightYInput);
DriveOutput arcadeDrive(double forwardInput, double turnInput);
DriveOutput curvatureDrive(double forwardInput, double turnInput);
DriveOutput computeDriveOutput(DriveMode mode, const JoystickInput& input);
```

Input should already have gone through `InputProfile`.

| Mode | Sticks | How it feels |
| --- | --- | --- |
| **Tank** | Left Y → left side, right Y → right side | Direct control of each side |
| **Arcade** | Left Y = forward, right X = turn | The most common choice |
| **Curvature** | Left Y = forward, right X = turn | Turning gets gentler at high speed; spins in place when stopped |

Details:

* **Arcade** clips each side to ±1. At full forward plus a turn, the faster side
  can't go any faster, so the robot turns less than you might expect.
* **Curvature** multiplies turning by how fast you're going forward, so small
  steering corrections stay small at speed. When forward is nearly 0, it switches
  to spinning in place, because otherwise it couldn't turn while stopped.

  > ⚠️ The "nearly 0" check (0.05) happens *after* the deadband and curve. With
  > the default settings, the robot keeps spinning in place until the forward
  > stick is past about 26%, then turning suddenly gets much weaker
  > ([code review #12](../code-review.md#12-curvature-drive-mode-pivots-until-a-quarter-stick)).

**To add a mode:** add a value to the `enum` and a `case` in
`computeDriveOutput()`. Nothing else needs to change.

---

## Acceleration limits

**`AccelLimitResolver`** decides how quickly driver speed may change, based on
[flags](subsystem.md#flagregistry). This is how the robot automatically drives
more gently while an arm is raised.

```cpp
struct AccelLimitRule {
    const char* flagName;        // the flag to check
    double      maxRpmPerSecond; // limit while that flag is true
};

struct AccelLimitConfig {
    double defaultMaxRpmPerSecond;
    std::vector<AccelLimitRule> rules;
};

explicit AccelLimitResolver(const AccelLimitConfig& config);

double resolve() const;   // cheap enough to call every loop
```

**How it picks:** every rule whose flag is true offers its limit, and the
**lowest** (strictest) wins. If no flags are true, it uses the default.

`opcontrol()` applies the result to both sides every loop:

```cpp
const double accelLimit = gAccelLimitResolver->resolve();
gLeftAccelSlew->setMaxRate(accelLimit);
gRightAccelSlew->setMaxRate(accelLimit);
```

* Create the resolver **after** the subsystems whose flags it uses. When it's
  created, it warns about any flag name that isn't registered yet.
* The limit also applies to **slowing down**. With a 400 RPM/s limit, letting go
  of the stick at full speed takes roughly a second or more to stop.

---

## Button macros

**`ButtonMacroRunner`** lets one button press run a short sequence of mechanism
actions — for example "raise the arm, wait until it's there, lower it." It moves
forward **at most one step per loop** and **never waits**, so the driver keeps
control of the robot the whole time.

```cpp
struct MacroStep {
    std::function<void()> action;                      // runs once when the step starts
    std::function<bool()> isDone = []{ return true; }; // checked each loop; true = next step
};

struct ButtonMacro {
    const char* name;
    std::vector<MacroStep> steps;
};

class ButtonMacroRunner {
    void trigger(const ButtonMacro& macro);   // start (or restart) a macro
    void update();                            // call every loop, always
    bool isRunning() const;
};
```

### Why not just use the autonomous helpers?

`auton::waitUntilSettled()` **waits** — it doesn't return until done. That's fine
in autonomous. In `opcontrol()` it would freeze the whole driver loop, including
driving, until the macro finished.

### How it behaves

* **One macro at a time.** Starting a new one replaces the current one.
* **Pressing the same button again restarts it** from the first step.
* A step without `isDone` moves on right after its action.
* Call `update()` every loop, even when nothing is running.

Use `isRunning()` to stop other buttons from fighting the macro:

```cpp
if (!gButtonMacroRunner.isRunning()) {
    if (master.get_digital_new_press(DIGITAL_R1)) { gExampleArm->moveToPreset("HIGH"); }
    if (master.get_digital_new_press(DIGITAL_R2)) { gExampleArm->moveToPreset("LOW"); }
}
if (master.get_digital_new_press(DIGITAL_L1)) { gButtonMacroRunner.trigger(*gDemoArmMacro); }
gButtonMacroRunner.update();
```

> ⚠️ **Steps have no timeout.** If a step's `isDone` never becomes true (for
> example, the arm faults), the macro runs forever and the code above ignores R1/R2
> for the rest of the match
> ([code review #6](../code-review.md#6-a-button-macro-can-lock-out-the-arm-buttons)).
> Make `isDone` also accept failure states.

### Writing a macro

```cpp
ButtonMacro makeLiftCycleMacro(MyLift& lift) {
    return ButtonMacro{
        .name = "liftCycle",
        .steps = {
            MacroStep{
                .action = [&lift] { lift.moveToPreset("HIGH"); },
                .isDone = [&lift] {
                    const auto s = lift.getState();
                    return s == MyLiftState::holding || s == MyLiftState::faulted || s == MyLiftState::idle;
                },
            },
            MacroStep{
                .action = [&lift] { lift.moveToPreset("LOW"); },
                // no isDone: finishes right away
            },
        },
    };
}
```

The macro object must stay alive while it runs — `trigger()` keeps a pointer to
it. In `main.cpp`, macros are stored in globals.

`driver::demo::makeDemoArmCycleMacro()` is the same idea for the demo arm, and
should be removed along with it.

---

## Settings — `driver_control_constants.hpp`

```cpp
inline constexpr DriveMode kDriveMode = DriveMode::arcade;

inline const InputProfileConfig kInputProfileConfig{
    .curveExponent = 2.0,
    .deadband = 0.05,
};

inline constexpr double kMaxDriveRpm = 343.0;   // ⚠️ should be the cartridge RPM

inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 2000.0,   // basically no limit
    .rules = {
        AccelLimitRule{ .flagName = "exampleArm.isExtended", .maxRpmPerSecond = 400.0 },
    },
};
```

> ⚠️ All placeholders.
> * `kDriveMode`, `curveExponent`, and `deadband` are up to your driver.
> * `kMaxDriveRpm` should be the **motor cartridge RPM** (600 blue, 200 green,
>   100 red). At 343, full stick only reaches about 57% of top speed on blue
>   motors ([code review #1](../code-review.md#1-drivetrain-speed-units-disagree--driver-control-is-capped-at-57-speed)).
> * The one acceleration rule uses the demo arm's flag. Replace it with real ones.

---

## Watching it work

`opcontrol()` prints:

* a line **whenever the acceleration limit changes**, and
* a **status line once a second** with target speeds, ramped speeds, the current
  limit, and the arm's state.

Keep `pros terminal` open. Those lines are the easiest way to confirm the
acceleration limit is really switching when the arm moves.
