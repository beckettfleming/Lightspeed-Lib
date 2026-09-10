---
title: Control
parent: Layers
nav_order: 2
permalink: /layers/control/
---

**Namespace:** `lightspeed::control` · **Headers:** `include/lightspeed/control/`

This layer turns *targets* into *voltages*. It contains three things: a generic PIDF controller,
a generic slew-rate limiter, and the drivetrain velocity controller built from both.

Everything that moves the drivetrain — driver control and every autonomous primitive — ends up
here. See the [single-output-path rule]({{ site.baseurl }}/architecture/#the-single-output-path-rule).

---

## `PIDFController`

A generic feedback + feedforward controller. It has **no notion of position or velocity** — it
just tracks a measurement against a setpoint. That's why the same class backs both the
drivetrain's velocity loop and every subsystem's position loop, with different gains supplied
per use.

### Configuration

```cpp
struct PIDFConfig {
    double kP, kI, kD;        // feedback gains
    double kV;                // feedforward: output per unit of target velocity
    double kA;                // feedforward: output per unit of target acceleration
    double kS;                // feedforward: output to overcome static friction
    double integralZone;      // only integrate when |error| is inside this; <= 0 disables gating
    double integralMax;       // hard clamp on the accumulator; <= 0 disables
    double settleTolerance;   // |error| threshold for "settled"
    std::uint32_t settleCycles; // consecutive cycles inside tolerance before isSettled() is true
};
```

### The feedforward terms, plainly

Feedback (PID) reacts to error *after* it appears. Feedforward predicts what output you'll need
*before* the error shows up, so PID only has to correct the leftovers.

| Term | Meaning | When it matters |
|---|---|---|
| **kV** | "How much output per unit of speed I want" | The main term for velocity control — do most of the work here, not in kP |
| **kA** | "How much extra output while accelerating" | Aggressive motion profiles |
| **kS** | "How much output just to break static friction" | Applied as `kS * sign(targetVelocity)`. Fixes the robot not starting on tiny commands |

### Usage

```cpp
control::PIDFController pidf(config);

control::Setpoint setpoint{
    .target = 200.0,             // position OR velocity — the class has no opinion
    .targetVelocity = 0.0,
    .targetAcceleration = 0.0,
};

double output = pidf.calculate(measurement, setpoint, dtSeconds);  // once per cycle

if (pidf.isSettled()) { /* within tolerance for settleCycles in a row */ }

pidf.reset();   // clear integral, derivative history, settle counter
```

**Call `reset()` when switching to a new setpoint** that shouldn't inherit old accumulated
state. `Subsystem::transitionTo()` does this automatically on every state change.

### Integral windup protection

Two independent guards, usable together:

- **`integralZone`** — only accumulate when the error is already small. Prevents a big integral
  building up during a long approach.
- **`integralMax`** — a hard clamp on the accumulator regardless of zone.

Set both to `0.0` to disable them. The drivetrain config currently ships with `kI = 0.0` and
`integralZone = 0.0` — integral is intentionally off until kP and kD are tuned.

---

## `SlewRateLimiter`

Limits how fast a value can change. Standalone and independent of `PIDFController`, so it's
reusable anywhere something needs ramping instead of stepping.

```cpp
control::SlewRateLimiter slew(240000.0);   // max change per second

double limited = slew.calculate(target, dtSeconds);  // once per cycle

slew.setMaxRate(newRate);   // swappable at runtime
slew.reset(currentValue);   // snap without ramping
```

The **runtime-swappable rate** is the point. Driver control uses it to tighten acceleration
while a mechanism is extended — see [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/).

`reset(value)` snaps the internal output without ramping, for re-enabling a controller that
shouldn't ramp up from whatever the last output happened to be.

The class is used in two distinct places, in different units:

| Where | Limits | Units |
|---|---|---|
| `DrivetrainVelocityController` | commanded **voltage** | mV/s |
| Driver control (`opcontrol`) | commanded **velocity** | RPM/s |

---

## `DrivetrainVelocityController`

The heart of the drivetrain. Takes a target RPM per side and runs a **100 Hz background task**
that drives the motors there, with feedforward, PID trim, slew limiting, battery-sag
compensation, and motor-health fail-safes.

### Using it

The entire public interface is one method:

```cpp
control::DrivetrainVelocityController drivetrain(
    leftMotorGroup, rightMotorGroup, control::kDrivetrainVelocityConfig);

drivetrain.setTargetVelocity(leftRpm, rightRpm);   // safe from any task
```

`setTargetVelocity()` writes two `std::atomic<double>` values and returns immediately. The
control task picks up the latest values on its next cycle. There is no locking and no blocking,
which is why it's safe to call at 100 Hz from `opcontrol()` and from any motion primitive.

The class is **non-copyable and non-movable** — it owns a background task that holds a `this`
pointer.

### What one control cycle does

```
  every 10 ms, for each side:
        |
        v
  read target RPM (atomic)  and  measured RPM (from hal::MotorGroup)
        |
        v
  check motor health  ------> stalled / over-temp / all-disconnected?
        |                          |
        | healthy                  v
        |                     zero output, reset PIDF integrator
        |                     and slew limiter, publish health
        v
  PIDF: kV * target + kS * sign(target) + kP/kI/kD on (target - measured)
        |
        v
  battery-sag compensation:  output *= nominalMv / actualMv   (capped at 1.0x below 11 V)
        |
        v
  slew-rate limit the commanded voltage (mV/s)
        |
        v
  clamp to +/-12000 mV  ->  hal::MotorGroup::writeVoltage()
        |
        v
  publish target / actual / health / connected-motor-count to the telemetry bus
```

### Battery-sag compensation

Gains are tuned against a reference pack voltage (`nominalBatteryMillivolts`, 12000 mV). As the
pack sags under load, the same voltage command produces less torque. The controller scales
output by `nominal / actual` so behavior stays consistent.

Below `lowBatteryMillivolts` (11000 mV) the compensation **stops boosting further** — it's
capped at 1.0×, **not** cut below it. That's deliberate: a critically low pack still needs to
drive off the field, not be stranded. The `drivetrain.batteryLow` telemetry channel records
when this is active.

### Motor-health fail-safe

| Health | Response |
|---|---|
| `stalled` | Zero output; reset the PIDF integrator and slew limiter |
| `overTemperature` | Zero output; reset the PIDF integrator and slew limiter |
| `disconnected`, **some** motors alive | **Keep driving** in a degraded state |
| `disconnected`, **all** motors gone | Zero output |

The partial-disconnect distinction uses `hal::MotorGroup::getConnectedMotorCount()`. Losing one
of two motors on a side shouldn't stop that side entirely.

> This was a real bug caught in review: the controller used to read health every cycle and only
> *record* it for telemetry. A stalled or overheating drive motor could be commanded
> indefinitely.

### Telemetry it publishes

| Channel | Type | Meaning |
|---|---|---|
| `drivetrain.left.targetRpm` / `.right.targetRpm` | number | Commanded velocity |
| `drivetrain.left.actualRpm` / `.right.actualRpm` | number | Measured velocity |
| `drivetrain.left.health` / `.right.health` | text | `OK` / `STALLED` / `OVER_TEMP` / `DISCONNECTED` |
| `drivetrain.left.connectedMotors` / `.right.connectedMotors` | integer | Motors currently responding |
| `drivetrain.batteryLow` | boolean | Below the low-battery threshold |

Comparing `targetRpm` against `actualRpm` in the SD log or serial plot is the single most
useful thing you can do while tuning. See [Tuning Guide]({{ site.baseurl }}/guides/tuning/).

---

## Configuration

[`include/lightspeed/control/drivetrain_velocity_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/control/drivetrain_velocity_constants.hpp)

```cpp
inline const DrivetrainVelocityConfig kDrivetrainVelocityConfig{
    .pidf = {
        .kP = 20.0,               // mV per RPM of error          TODO: tune
        .kI = 0.0,                //                              TODO: tune after kP/kD
        .kD = 0.0,                //                              TODO: tune
        .kV = 35.0,               // mV per target RPM (~12000/343) TODO: tune
        .kA = 0.0,                // unused
        .kS = 300.0,              // mV to break static friction  TODO: tune
        .integralZone = 0.0,      // disabled until kI is tuned
        .integralMax = 4000.0,    // mV windup clamp
        .settleTolerance = 10.0,  // RPM
        .settleCycles = 10,       // ~100 ms at 100 Hz
    },
    .maxVoltageSlewRatePerSecond = 240000.0,  // full 12 V swing in ~50 ms  TODO: tune
    .nominalBatteryMillivolts = 12000.0,
    .lowBatteryMillivolts = 11000.0,
    .loopPeriodMs = 10,   // 100 Hz — matches the V5 motor's own update rate
};
```

> ⚠️ **Every gain here is a placeholder.** They cannot be simulated correctly without the real
> drivetrain's mass, friction, and battery behavior. Tune them with the wheels off the ground
> before ever driving. See [Tuning Guide]({{ site.baseurl }}/guides/tuning/).

**Don't raise `loopPeriodMs` above 100 Hz.** The V5 motor's own internal update rate is 100 Hz;
running the loop faster gains nothing and just burns CPU.

---

**Next:** [Odometry Layer]({{ site.baseurl }}/layers/odometry/) — figuring out where the robot is.
