---
title: Driver Control
parent: Layers
nav_order: 5
permalink: /layers/driver-control/
---

**Namespace:** `lightspeed::driver` · **Headers:** `include/lightspeed/driver/`

Turns joystick positions into drivetrain velocity targets, with an input curve, a swappable
drive scheme, condition-driven acceleration limiting, and non-blocking button macros.

---

## The pipeline

`opcontrol()` runs this at **100 Hz**:

```
  Controller sticks (-127 .. 127)
        |
        v
  normalize to -1 .. 1
        |
        v
  InputProfile      deadband, then expo curve
        |
        v
  computeDriveOutput()   tank / arcade / curvature  ->  left, right in -1 .. 1
        |
        v
  x kMaxDriveRpm    ->  target RPM per side
        |
        v
  AccelLimitResolver  ->  current max RPM/s, from active subsystem flags
        |
        v
  SlewRateLimiter (per side)   caps how fast the target may change
        |
        v
  DrivetrainVelocityController::setTargetVelocity(left, right)
```

Every stage is a separate, testable object. Nothing in the pipeline writes voltage — that's the
control layer's job.

---

## Controller map

| Input | Action |
|---|---|
| **Left stick Y** | Forward / back (arcade and curvature); left side (tank) |
| **Right stick X** | Turn (arcade and curvature) |
| **Right stick Y** | Right side (tank mode only) |
| **L1** *(hold)* | Front intake forward + rear intake reverse |
| **L1** *(press)* | **Also** triggers the demo arm-cycle macro |
| **B** *(hold)* | Both intakes forward |
| **Down** *(hold)* | Both intakes reverse |
| **R1** *(press)* | Demo arm to `HIGH` |
| **R2** *(press)* | Demo arm to `LOW` |
| **Y** *(held at boot)* | Enter [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/) instead of normal operation |

> ⚠️ **L1 is currently double-bound**: it both runs the intakes (hold) and triggers the demo
> macro (new press). That's a leftover from the demo macro being bound before the intake
> controls existed. Rebind one of them in `opcontrol()` once you have real mechanisms — the
> macro binding is the one to move, since the demo arm is itself a placeholder.

Intake controls are **hold-to-run**, so releasing immediately stops both motors. L1 takes
priority for opposing motion.

Direct arm buttons (R1/R2) are **suppressed while a macro is running**, so a direct press can't
fight the macro's own commands.

---

## `InputProfile`

Deadband plus expo curve on a single axis.

```cpp
struct InputProfileConfig {
    double curveExponent = 1.0;   // 1.0 = linear; > 1.0 = more low-speed precision
    double deadband = 0.0;        // ignore |input| below this, in [0, 1)
};

driver::InputProfile profile(driver::kInputProfileConfig);
double out = profile.apply(rawInput);   // clamped to [-1,1], sign-preserving
```

**Deadband** ignores tiny stick values so a slightly off-center stick doesn't creep the robot.

**Curve exponent** compresses the low end while leaving the endpoints (`-1`, `0`, `1`) fixed. A
higher exponent means finer control at low speed with no loss of top speed. `1.3` is the current
default — a gentle curve. Raise it if the driver wants softer low-speed response.

Config is swappable at runtime via `setConfig()`.

---

## Drive modes

```cpp
enum class DriveMode { tank, arcade, curvature };
```

Selected by `kDriveMode` in `driver_control_constants.hpp` (**currently `arcade`**).

### Tank

Each side driven directly by its own stick. Uses `leftY` and `rightY`.

### Arcade (split arcade)

`leftY` drives both sides together; `rightX` steers differentially. Each side is clamped to
`[-1, 1]`, so turning at full forward can't exceed a side's own maximum.

```
left  = clamp(forward + turn, -1, 1)
right = clamp(forward - turn, -1, 1)
```

Predictable and the usual default.

### Curvature ("cheesy") drive

The turn contribution **scales with `|forward|`**:

```
turnContribution = turn * |forward|
left  = clamp(forward + turnContribution, -1, 1)
right = clamp(forward - turnContribution, -1, 1)
```

At high speed, small stick movements make gentle corrections instead of arcade's always-full
differential steering. Many drivers find it much smoother for long straight runs.

**The catch, and how it's handled:** pure curvature steering can't turn in place — as forward
goes to zero, so does the turn contribution. Below `|forward| < 0.05` the implementation falls
back to a direct tank-style pivot, so the robot can still rotate from a standstill.

### Adding a mode

Add an enumerator, write the transform function, and add a `case` to `computeDriveOutput()`.
Call sites don't change.

---

## Acceleration limiting

This is the feature that makes driver control aware of the rest of the robot.

### How it works

```cpp
struct AccelLimitRule {
    const char* flagName;         // read from subsystem::FlagRegistry
    double maxRpmPerSecond;
};

struct AccelLimitConfig {
    double defaultMaxRpmPerSecond;
    std::vector<AccelLimitRule> rules;
};
```

Every cycle, `AccelLimitResolver::resolve()` checks each rule's flag. **Every rule whose flag is
currently `true` contributes its value, and the minimum — the most restrictive — wins.** No true
flags means the configured default.

```cpp
const double accelLimit = accelResolver.resolve();
leftSlew.setMaxRate(accelLimit);
rightSlew.setMaxRate(accelLimit);
const double leftSlewedRpm = leftSlew.calculate(leftTargetRpm, dtSeconds);
```

### Why you want it

A robot with its lift extended has a high center of gravity. Slamming the sticks tips it over.
The rule table says "while `lift.isExtended` is true, cap acceleration at 400 RPM/s" — and the
driver simply cannot tip the robot no matter how hard they push the stick. It's automatic, and
the driver doesn't have to think about it.

### Current configuration

```cpp
inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 12000.0,   // effectively unlimited  TODO: tune for feel
    .rules = {
        AccelLimitRule{.flagName = "exampleArm.isExtended", .maxRpmPerSecond = 400.0},
    },
};
```

The `exampleArm` rule is a stand-in demonstrating the mechanism. Replace it with real
subsystem flags once real mechanisms exist.

### Seeing it work

`opcontrol()` prints whenever the resolved limit changes:

```
[opcontrol] accel limit changed: 12000 -> 400 RPM/s (exampleArm.isExtended=true)
```

Press R1 to raise the demo arm past its threshold and watch the line appear. That's the proof
the whole flag-registry → resolver → slew-limiter chain is wired end to end, not just
structurally present.

---

## Button macros

Binds a button to a short scripted sequence of subsystem actions, advanced **one step per
opcontrol tick** — never blocking.

### Why not reuse the autonomous sequencer

`auton::waitUntilSettled()` blocks the calling task. That's fine for the autonomous task, which
has nothing else to do. In `opcontrol()` it would stall the **entire 100 Hz loop** — including
the drivetrain's own `setTargetVelocity()` calls — for the macro's whole duration. The driver
would lose control of the robot while the macro ran.

So `ButtonMacroRunner` is a per-tick state machine instead.

### Defining a macro

```cpp
struct MacroStep {
    std::function<void()> action;              // runs once, when the step becomes active
    std::function<bool()> isDone = []{ return true; };  // polled each tick while active
};

struct ButtonMacro {
    const char* name;
    std::vector<MacroStep> steps;
};
```

A step with no `isDone` advances immediately after its action — useful for a final
"fire this and finish" step.

```cpp
ButtonMacro makeArmCycleMacro(Lift& lift) {
    return ButtonMacro{
        .name = "Arm cycle",
        .steps = {
            MacroStep{
                .action = [&lift] { lift.moveToPreset("HIGH"); },
                .isDone = [&lift] { return lift.getState() == LiftState::holding; },
            },
            MacroStep{
                .action = [&lift] { lift.moveToPreset("LOW"); },
                // default isDone: fire and finish
            },
        },
    };
}
```

### Running it

```cpp
if (master.get_digital_new_press(DIGITAL_L1)) {
    macroRunner.trigger(demoMacro);
}
macroRunner.update();          // call EVERY tick, macro running or not
```

| Method | Behavior |
|---|---|
| `trigger(macro)` | Starts from step 0, replacing any macro in progress |
| `update()` | Advances the active macro by at most one step; no-op when idle |
| `isRunning()` | True between `trigger()` and the last step completing |

Only **one macro runs at a time**. Re-triggering the same macro **restarts** it from step 0
rather than resuming — a second press generally means "do it again", not "continue".

Use `isRunning()` to suppress conflicting direct bindings, exactly as `opcontrol()` does for
R1/R2.

---

## Configuration

[`include/lightspeed/driver/driver_control_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/driver/driver_control_constants.hpp)

```cpp
inline constexpr DriveMode kDriveMode = DriveMode::arcade;   // TODO: driver preference

inline const InputProfileConfig kInputProfileConfig{
    .curveExponent = 1.3,   // TODO: tune -- higher = softer low-speed response
    .deadband = 0.05,
};

inline constexpr double kMaxDriveRpm = 343.0;   // must match the drivetrain kV assumption

inline const AccelLimitConfig kDriveAccelLimitConfig{
    .defaultMaxRpmPerSecond = 12000.0,   // TODO: tune for driver feel
    .rules = { /* ... */ },
};
```

> **`kMaxDriveRpm` and `control::kDrivetrainVelocityConfig.pidf.kV` describe the same physical
> top speed.** Change one and you must change the other. See [Configuration Reference]({{ site.baseurl }}/reference/configuration/).

**Which drive mode to use is a driver-preference call**, which is why `curvature` exists but
isn't the default. Let your driver try all three.

---

**Next:** [Motion Layer]({{ site.baseurl }}/layers/motion/) — moving the robot autonomously.
