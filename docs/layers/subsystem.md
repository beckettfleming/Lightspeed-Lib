---
title: Subsystem
parent: Layers
nav_order: 4
permalink: /layers/subsystem/
---

**Namespace:** `lightspeed::subsystem` · **Headers:** `include/lightspeed/subsystem/`

A framework for robot mechanisms — lifts, intakes, arms. Each subsystem gets a state machine,
PIDF position control, named presets, automatic fault handling, and automatic telemetry, and is
updated by one shared scheduler task rather than spawning its own.

---

## The four pieces

| Piece | What it is |
|---|---|
| `SchedulableSubsystem` | The non-template interface the scheduler stores and calls |
| `Subsystem<StateEnum>` | The templated base class you actually inherit from |
| `Scheduler` | One 50 Hz task that calls `update()` on everything registered |
| `FlagRegistry` | A named boolean store so subsystems can publish conditions |

### Why two base classes

Each concrete subsystem supplies its own state enum type, so `Subsystem<StateEnum>` is a
template — and templates with different arguments are unrelated types that can't live in one
container. `SchedulableSubsystem` is the plain, non-template interface that `Subsystem<T>`
implements, which is what lets the scheduler hold a `Lift` and an `Intake` in the same array.

`Subsystem<StateEnum>` is header-only for the same reason: there's no single set of template
arguments to explicitly instantiate in a `.cpp`.

---

## `Scheduler`

A singleton owning one PROS task at **50 Hz (20 ms)**.

```cpp
subsystem::Scheduler::instance().start();   // call once, after constructing subsystems
```

Subsystems **register themselves automatically** in the `Subsystem<StateEnum>` constructor — you
never call `registerSubsystem()` by hand. Registration is safe from any task, including after
`start()` (a late arrival is simply picked up on the next cycle).

- Maximum **8** subsystems (`kMaxSubsystems`)
- `start()` is a no-op if already started

**Every `update()` must be fast and non-blocking.** They all share one task, so a busy-wait in
one subsystem stalls every other subsystem on the robot.

---

## `Subsystem<StateEnum>`

### Configuration

```cpp
struct Preset {
    const char* name;
    double positionRaw;    // raw HAL units, e.g. motor degrees
};

struct SubsystemConfig {
    const char* name;
    control::PIDFConfig pidf;
    std::vector<Preset> presets;
};
```

### What you get

| Method | Access | Purpose |
|---|---|---|
| `getState()` | public | Current state — safe from any task |
| `getName()` | public | The subsystem's name |
| `update(dt)` | `final` | Called by the scheduler; you don't override this |
| `onUpdate(dt)` | **pure virtual** | Your per-cycle logic |
| `onEnter(state)` / `onExit(state)` | virtual | React to state transitions |
| `onFault(status)` | virtual | React to a motor health problem |
| `transitionTo(state)` | protected | Change state (runs exit/enter hooks, resets PIDF) |
| `driveToPosition(target, dt)` | protected | PIDF position control with a built-in fail-safe |
| `getPresetPosition(name)` | protected | Look up a preset; `nullopt` if unknown |
| `registerFlag()` / `setFlag()` | protected | Publish conditions to the flag registry |
| `lock()` / `unlock()` | protected | The internal recursive mutex, for your own state |

### `update()` is `final` — override `onUpdate()`

`update()` is sealed so it can wrap your whole cycle in the subsystem's lock and publish
telemetry afterward:

```cpp
void update(double dtSeconds) final {
    mutex_.take();
    onUpdate(dtSeconds);                                    // your logic
    telemetry::TelemetryBus::instance()
        .record(name_, static_cast<std::int32_t>(state_));  // automatic
    mutex_.give();
}
```

Two things come free: your cycle can't be interrupted mid-way by an external command, and every
subsystem's state ordinal reaches the telemetry bus with **zero per-subsystem wiring**.

### State transitions

```cpp
transitionTo(MyState::holding);
```

1. `onExit(oldState)`
2. State changes
3. **`pidf_.reset()`** — fresh integral, derivative, and settle history
4. `onEnter(newState)`

A same-state transition is a no-op. The mutex is **recursive**, so calling `transitionTo()` from
inside `onUpdate()` (e.g. via `onFault()`) is safe.

### `driveToPosition()` and its fail-safe

```cpp
void driveToPosition(double targetPositionRaw, double dtSeconds);
```

Before doing anything, it checks motor health and **stops unconditionally** when:

- health is `stalled`, **or**
- health is `overTemperature`, **or**
- health is `disconnected` **and every motor in the group is gone**

On stop it writes 0 V and calls `onFault(health)` — but the zeroing happens **whether or not
your `onFault()` reacts.**

> This was a real bug caught in review. The base class used to rely entirely on `onFault()` to
> react. `ExampleArm`'s override only handles `stalled`, so an over-temperature or disconnected
> motor could leave stale voltage commanded indefinitely while the fault was re-detected every
> cycle forever.

A **partial** disconnect on a multi-motor mechanism keeps driving in a degraded state rather
than stopping something still partially usable.

Otherwise it runs PIDF position control against `motors_.getPositionRaw()` and writes the
clamped output.

Call it **only from `onUpdate()`** — the lock is already held there.

### Thread safety in your own subsystem

`getState()` and `transitionTo()` are guarded for you. **Anything you add yourself is not.**
Guard your own cross-task state with `lock()` / `unlock()`:

```cpp
void Lift::moveToPreset(const char* presetName) {
    const auto position = getPresetPosition(presetName);
    if (!position.has_value()) { return; }

    lock();
    targetPositionRaw_ = *position;              // your own member
    transitionTo(LiftState::movingToTarget);     // re-enters the same recursive lock
    unlock();
}
```

The lock makes the pair atomic — no scheduler cycle can observe the new target with the old
state or vice versa.

---

## `FlagRegistry`

A thread-safe named-boolean store. Subsystems publish conditions; other layers read them by
name without needing to know the subsystem's type.

```cpp
auto& flags = subsystem::FlagRegistry::instance();

flags.registerFlag("lift.isExtended", false);   // at init
flags.setFlag("lift.isExtended", true);         // from update()
bool extended = flags.getFlag("lift.isExtended", false);
```

| Method | Notes |
|---|---|
| `registerFlag(name, initial)` | `false` if already registered or the registry is full |
| `unregisterFlag(name)` | Swap-remove; for teardown and tests |
| `setFlag(name, value)` | `false` if never registered — catches typos |
| `getFlag(name, default)` | Returns `default` if never registered |
| `isRegistered(name)` | |
| `getRegisteredCount()` / `getNameAt(i)` | Enumeration, e.g. for a debug view |

Capacity is **16** flags. Names must be **stable-lifetime strings** (string literals) — the
registry stores the pointer, not a copy.

Every `setFlag()` also forwards into the telemetry bus, so flags show up in the SD log for free.

### The main consumer

Driver control's accel-limit table reads flags by name to decide how hard to cap acceleration.
See [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/).

### ⚠️ Known limitation: no ownership enforcement

Any caller that knows a flag's name can set it. `registerFlag()` rejects a *second* registration
of the same name (returns `false` without overwriting), but **if that return value is ignored**,
the second subsystem still believes it owns the name and will happily `setFlag()` it every
cycle, silently stomping the first subsystem's value.

With only one flag in the codebase today this is a documented risk, not a live bug. **Avoid it
by prefixing every flag with its subsystem's name** — `"lift.isExtended"`, `"intake.hasRing"` —
exactly as `exampleArm.isExtended` already does.

---

## `ExampleArm` — the reference implementation

> 🚧 **`ExampleArm` is a placeholder, not a real mechanism.** It exists to exercise the framework
> end to end on a single motor. Delete or replace it once real mechanisms are designed. Don't
> build on top of it as if it were real.

It is, however, the best worked example of the framework, so it's worth reading:
[`src/lightspeed/subsystem/demo/example_arm.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/subsystem/demo/example_arm.cpp)

### Its state machine

```
              moveToPreset(name)
                     |
                     v
  IDLE ------> MOVING_TO_TARGET ------> HOLDING
   ^ ^              |    ^                 |
   | |              |    +-----------------+
   | |              |      moveToPreset()
   | |    pidf_.isSettled()
   | |
   | |         any state, on a stall
   | +---------------+
   |                 v
   |             FAULTED
   +-----------------+
      health returns to ok
```

| State | Behavior |
|---|---|
| `idle` | Writes 0 V |
| `movingToTarget` | `driveToPosition(target)`; transitions to `holding` once the PIDF settles |
| `holding` | `driveToPosition(target)` — actively holds position against gravity |
| `faulted` | Writes 0 V; returns to `idle` once health reads `ok` again |

Its presets are `LOW` (0°), `MID` (90°), `HIGH` (180°) in raw motor degrees.

Every cycle it also publishes `exampleArm.isExtended` = position > 45°, which is what makes the
driver-control accel limiter visibly react.

`onEnter(holding)` latches the **current physical position** as the hold target rather than
trusting the target it was driving toward — a small but important detail for mechanisms that
don't quite reach their setpoint.

`simulateFault()` exists for bench testing: it forces the fault reaction without physically
jamming anything.

---

## Writing your own subsystem

```cpp
// include/lightspeed/subsystem/lift.hpp
#pragma once
#include "lightspeed/hal/motor_group.hpp"
#include "lightspeed/subsystem/subsystem.hpp"

namespace lightspeed::subsystem {

enum class LiftState { idle, movingToTarget, holding, faulted };
[[nodiscard]] const char* toString(LiftState state);

class Lift : public Subsystem<LiftState> {
public:
    explicit Lift(hal::MotorGroup& motors);
    void moveToPreset(const char* presetName);   // safe from any task

protected:
    void onUpdate(double dtSeconds) override;
    void onEnter(LiftState state) override;
    void onFault(hal::HealthStatus status) override;

private:
    double targetPositionRaw_ = 0.0;   // guarded by lock()/unlock()
};

}  // namespace lightspeed::subsystem
```

### Checklist

1. **Add the ports** to `hal/config.hpp` and define a `MotorGroupConfig`.
2. **Define a state enum** and a `toString()` for it.
3. **Write a `SubsystemConfig`** in your `.cpp` (anonymous namespace) with the name, PIDF gains,
   and presets.
4. **Register your flags** in the constructor, with a subsystem-name prefix.
5. **Implement `onUpdate()`** as a `switch` over `getState()`, calling `driveToPosition()` where
   position control is wanted.
6. **Implement `onFault()`** to transition to a fault state.
7. **Construct it in `initialize()`** — *before* `Scheduler::instance().start()` and before
   `AccelLimitResolver`, so its flags exist when those run.
8. **Add it to `AutonomousContext`** if autonomous routines need it.

You do **not** need to: register with the scheduler, publish state telemetry, handle the
motor-health fail-safe, or reset the PIDF on state change. All of that is inherited.

---

**Next:** [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/) — the joystick side.
