# `lightspeed::driver` — driver control

Everything between the joystick and
`control::DrivetrainVelocityController::setTargetVelocity()`. The pipeline
`opcontrol()` runs every 20 ms:

```
raw joystick  →  InputProfile  →  computeDriveOutput  →  × kMaxDriveRpm
              →  SlewRateLimiter (rate from AccelLimitResolver)
              →  DrivetrainVelocityController::setTargetVelocity()
```

Each stage is an independent, testable piece. Note that the output of this
pipeline is the **same call** autonomous routines make — there is one path to
the motors.

## `InputProfile`

Deadband, then expo curve, on a single normalized axis.

```cpp
struct InputProfileConfig {
    double curveExponent = 1.0;   // 1.0 = linear; > 1.0 = more low-speed precision
    double deadband = 0.0;        // ignore |input| below this, in [0, 1)
};

explicit InputProfile(const InputProfileConfig& config);

double apply(double rawInput) const;   // sign-preserving, output in [-1, 1]
void   setConfig(const InputProfileConfig& config);
const InputProfileConfig& getConfig() const;
```

Input is clamped to `[-1, 1]` first. The endpoints (`-1`, `0`, `1`) stay fixed
regardless of exponent — a higher exponent compresses the low end for precision
without giving up top-end response.

`opcontrol()` applies one profile instance to all four axes.

## Drive modes

```cpp
enum class DriveMode { tank, arcade, curvature };

struct JoystickInput { double leftY, leftX, rightY, rightX; };
struct DriveOutput   { double left, right; };   // normalized

DriveOutput tankDrive(double leftYInput, double rightYInput);
DriveOutput arcadeDrive(double forwardInput, double turnInput);
DriveOutput curvatureDrive(double forwardInput, double turnInput);
DriveOutput computeDriveOutput(DriveMode mode, const JoystickInput& input);
```

Input is expected to be **already profiled** by the caller.

* **Tank** — each side driven directly by its own stick.
* **Arcade** (split) — forward on one stick, turn on the other; output clamped to
  `[-1, 1]` per side, so turning at full forward cannot exceed a side's max.
* **Curvature** ("cheesy") — turn's contribution scales with `|forward|`, so
  small corrections stay gentle at speed instead of applying full differential
  steering the way arcade does. Below a forward magnitude of **0.05**, it falls
  back to a direct tank-style pivot so the robot can still rotate in place —
  without that, pure curvature steering cannot turn from a standstill, since
  turn's contribution scales toward zero as forward does.

Adding a fourth mode means adding an enum value and a `case` in
`computeDriveOutput()` — no call site changes.

## `AccelLimitResolver`

Maps [flag registry](subsystem.md#flagregistry) conditions to a maximum
acceleration for the driver-control slew limiter. This is the feature that lets
the drivetrain automatically get gentler while a mechanism is extended.

```cpp
struct AccelLimitRule {
    const char* flagName;        // queried from subsystem::FlagRegistry
    double      maxRpmPerSecond;
};

struct AccelLimitConfig {
    double defaultMaxRpmPerSecond;
    std::vector<AccelLimitRule> rules;
};

explicit AccelLimitResolver(const AccelLimitConfig& config);

double resolve() const;   // cheap — safe every cycle
```

**Every rule whose flag is currently true contributes its value, and the minimum
— the most restrictive — wins.** No true flags means the configured default.

`opcontrol()` feeds the result straight into both sides' slew limiters:

```cpp
const double accelLimit = gAccelLimitResolver->resolve();
gLeftAccelSlew->setMaxRate(accelLimit);
gRightAccelSlew->setMaxRate(accelLimit);
```

Construct the resolver **after** the subsystems whose flags its rules name, so
the flags are already registered when its sanity check runs.

## `ButtonMacroRunner`

Binds a button to a short scripted sequence of subsystem actions, advanced **one
step per opcontrol tick** — it never blocks.

```cpp
struct MacroStep {
    std::function<void()> action;                      // runs once, on becoming active
    std::function<bool()> isDone = []{ return true; }; // polled while active
};

struct ButtonMacro {
    const char* name;
    std::vector<MacroStep> steps;
};

class ButtonMacroRunner {
    void trigger(const ButtonMacro& macro);   // restarts from step 0
    void update();                            // call once per tick, always
    bool isRunning() const;
};
```

**Why this exists instead of reusing `auton::sequencer`.** The sequencer's
`waitUntilSettled()` blocks the calling task. That is fine on the autonomous
task. In `opcontrol()` it would stall the entire 50 Hz loop — including the
drivetrain's own `setTargetVelocity()` calls — for the macro's whole duration.
The driver would lose control of the robot while a macro ran.

Semantics worth knowing:

* Only **one macro at a time**. Triggering a new one replaces the active one; it
  does not queue or interleave.
* Re-triggering the **same** macro restarts it from step 0 — a second press
  generally means "do it again," not "resume."
* A step with no `isDone` advances immediately after its action — useful for a
  final fire-and-finish step.
* Call `update()` every tick unconditionally; it is a no-op when idle.

Use `isRunning()` to suppress conflicting direct bindings, as `opcontrol()` does:

```cpp
if (!gButtonMacroRunner.isRunning()) {
    if (master.get_digital_new_press(DIGITAL_R1)) { gExampleArm->moveToPreset("HIGH"); }
    if (master.get_digital_new_press(DIGITAL_R2)) { gExampleArm->moveToPreset("LOW"); }
}
if (master.get_digital_new_press(DIGITAL_L1)) { gButtonMacroRunner.trigger(*gDemoArmMacro); }
gButtonMacroRunner.update();
```

### Writing a macro

```cpp
ButtonMacro makeLiftCycleMacro(MyLift& lift) {
    return ButtonMacro{
        .name = "liftCycle",
        .steps = {
            MacroStep{
                .action = [&lift] { lift.moveToPreset("HIGH"); },
                .isDone = [&lift] { return lift.getState() == MyLiftState::holding; },
            },
            MacroStep{
                .action = [&lift] { lift.moveToPreset("LOW"); },
                // no isDone — finishes immediately after firing
            },
        },
    };
}
```

`driver::demo::makeDemoArmCycleMacro()` is exactly this shape against the demo
arm, and is a placeholder to delete alongside it.

## Constants — `driver_control_constants.hpp`

```cpp
inline constexpr DriveMode kDriveMode = DriveMode::arcade;

inline const InputProfileConfig kInputProfileConfig{
    .curveExponent = 2.0,
    .deadband = 0.05,
};

inline constexpr double kMaxDriveRpm = 343.0;

inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 2000.0,       // effectively "full send"
    .rules = {
        AccelLimitRule{ .flagName = "exampleArm.isExtended", .maxRpmPerSecond = 400.0 },
    },
};
```

> ⚠️ All placeholders. `kDriveMode`, the curve exponent, and the deadband are
> **driver-preference** calls — ask your driver rather than inheriting these.
> `kMaxDriveRpm` is tied to real gearing and must agree with
> `control::kDrivetrainVelocityConfig`'s `kV` and the gear ratios in `odom` and
> `motion` — see [Tuning § step 0](../tuning.md#step-0--make-the-geometry-true).
> The single accel-limit rule names the demo subsystem's flag and must be
> replaced along with it.

## Watching it work

`opcontrol()` prints a line whenever the resolved accel limit changes, plus a
1 Hz heartbeat with targets, slewed values, the current limit, and subsystem
state. Keep `pros terminal` open — those lines are how you confirm the
conditional accel system is genuinely wired end-to-end and not merely present.
