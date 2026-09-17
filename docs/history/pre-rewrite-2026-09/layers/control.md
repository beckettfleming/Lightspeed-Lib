# `lightspeed::control` — closed-loop control

Three classes: a generic PIDF controller, a standalone slew-rate limiter, and
the drivetrain velocity controller that composes both on top of two
`hal::MotorGroup`s.

The design decision underneath this layer: **Lightspeed uses voltage-based
control** (`move_voltage` plus its own feedforward and PID) rather than PROS's
built-in `move_velocity`. That means the gains here are the ones that determine
how the robot actually drives.

## `PIDFController`

Deliberately has **no notion of position or velocity**. It tracks a measurement
against a setpoint, so the same class backs the drivetrain's velocity loop and a
subsystem's position loop, with gains supplied per use.

```cpp
struct PIDFConfig {
    double kP, kI, kD;
    double kV;   // feedforward: output per unit of target velocity
    double kA;   // feedforward: output per unit of target acceleration
    double kS;   // feedforward: static friction, applied as kS * sign(targetVelocity)

    double integralZone;   // <= 0 disables zone-gating (always integrate)
    double integralMax;    // <= 0 disables the hard clamp on accumulated integral

    double        settleTolerance;
    std::uint32_t settleCycles;
};

struct Setpoint {
    double target;               // position or velocity — the class has no opinion
    double targetVelocity;
    double targetAcceleration;
};
```

```cpp
explicit PIDFController(const PIDFConfig& config);

double calculate(double measurement, const Setpoint& setpoint, double dtSeconds);
void   reset();          // clears integral, derivative history, settle counter
bool   isSettled() const;
void   setConfig(const PIDFConfig& config);
const PIDFConfig& getConfig() const;
```

Call `calculate()` once per control cycle. Call `reset()` whenever you switch to
a new setpoint that should not inherit accumulated state — the subsystem base
class does this automatically on every state transition.

`isSettled()` reports true once `|error| <= settleTolerance` for
`settleCycles` **consecutive** calls. Both halves matter: a tolerance with no
cycle count will trigger on a single noisy sample passing through the target.

**Windup guards.** `integralZone` gates integration to within that distance of
the target; `integralMax` hard-clamps the accumulator. Either can be used alone
or both together.

## `SlewRateLimiter`

Rate-of-change limiting, kept separate from `PIDFController` so it can be reused
anywhere a value should ramp rather than step.

```cpp
explicit SlewRateLimiter(double maxRatePerSecond);

double calculate(double target, double dtSeconds);
void   setMaxRate(double maxRatePerSecond);
double getMaxRate() const;
void   reset(double value = 0.0);   // snap without ramping
```

The max rate is **swappable at runtime**, which is the whole point: driver
control tightens it while a mechanism is extended (see
[driver § accel limiting](driver.md#accellimitresolver)), and the drivetrain
controller resets it when a fault zeroes output.

## `DrivetrainVelocityController`

Owns a **~100 Hz background task** driving both sides of the drivetrain to
target RPM. 100 Hz matches the V5 motor's own internal update rate; running
faster gains nothing.

```cpp
struct DrivetrainVelocityConfig {
    PIDFConfig    pidf;
    double        maxVoltageSlewRatePerSecond;  // mV/s
    double        nominalBatteryMillivolts;     // reference the gains were tuned at
    double        lowBatteryMillivolts;         // below this, stop boosting
    std::uint32_t loopPeriodMs;                 // 10 ms
};
```

```cpp
DrivetrainVelocityController(hal::MotorGroup& left, hal::MotorGroup& right,
                             const DrivetrainVelocityConfig& config);

void setTargetVelocity(double leftRpm, double rightRpm);   // safe from any task
```

Non-copyable and non-movable — it owns a task referencing `this`.

**`setTargetVelocity()` is the funnel point for the entire library.** Driver
control, every motion primitive, and every autonomous routine end here. Nothing
else writes voltage to the drive.

### Battery-sag compensation

Output is scaled by `nominalBatteryMillivolts / actualBatteryMillivolts`, so
behavior does not drift as the pack sags. Below `lowBatteryMillivolts`,
compensation stops boosting further — it is **capped at 1.0×, not cut below
it**. A critically low pack still needs to drive off the field; this is a sanity
limit on the multiplier, not a torque cutoff.

### Health fail-safes

Each cycle, per side:

* `stalled` or `overTemperature` → zero output, reset the PIDF integrator and
  the slew limiter.
* `disconnected` → zero output **only if every motor in the group is gone**. A
  partial disconnect keeps driving in a degraded state rather than stopping a
  side that is still partially drivable.

This was a real bug the review pass found: health was read every cycle but only
recorded for telemetry, so a stalled or overheating drive motor could be
commanded indefinitely.

## Constants — `drivetrain_velocity_constants.hpp`

```cpp
inline const DrivetrainVelocityConfig kDrivetrainVelocityConfig{
    .pidf = {
        .kP = 20.0,     // mV per RPM of error
        .kI = 0.0,
        .kD = 0.0,
        .kV = 35.0,     // mV per target RPM, ≈ 12000 mV / 343 rpm
        .kA = 0.0,
        .kS = 300.0,    // mV to overcome static friction
        .integralZone = 0.0,
        .integralMax = 4000.0,
        .settleTolerance = 10.0,   // RPM
        .settleCycles = 10,        // ~100 ms at 100 Hz
    },
    .maxVoltageSlewRatePerSecond = 40000.0,   // full 12 V swing in ~300 ms
    .nominalBatteryMillivolts = 12000.0,
    .lowBatteryMillivolts = 11000.0,
    .loopPeriodMs = 10,
};
```

> ⚠️ Every gain is a placeholder. `kV`'s 343 RPM assumption must agree with
> `driver::kMaxDriveRpm` and the gear ratios in `odom` and `motion` — see
> [Tuning § step 0](../tuning.md#step-0--make-the-geometry-true).
>
> `lowBatteryMillivolts` at 11 V is a reasonable generic V5-pack default, not a
> placeholder — leave it unless real match data says otherwise.

## Telemetry published

| Channel | Type |
| --- | --- |
| `drivetrain.left.targetRpm` / `.right.targetRpm` | number |
| `drivetrain.left.actualRpm` / `.right.actualRpm` | number |
| `drivetrain.left.health` / `.right.health` | text |
| `drivetrain.left.connectedMotors` / `.right.connectedMotors` | integer |
| `drivetrain.batteryLow` | boolean |
