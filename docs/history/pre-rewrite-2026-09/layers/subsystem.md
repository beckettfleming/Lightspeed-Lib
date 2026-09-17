# `lightspeed::subsystem` — subsystem framework

A templated base class giving every mechanism the same skeleton — state machine,
PIDF position control, named presets, fail-safe fault handling — plus a shared
scheduler that updates all of them from one task, and a registry of named
booleans other layers can react to.

## `Scheduler`

One PROS task at **~50 Hz (20 ms)** iterates every registered subsystem and calls
`update()`. Subsystems do **not** each run their own task.

```cpp
static Scheduler& instance();

void registerSubsystem(SchedulableSubsystem& subsystem);
void start();   // no-op if already started
```

Registration is automatic — `Subsystem`'s constructor does it. Call `start()`
once in `initialize()`, after every subsystem that should run this session has
been constructed. Registering after `start()` is safe; the new subsystem is
picked up on the next cycle.

Capacity: **8 subsystems** (`kMaxSubsystems`).

Because one task serves all of them, **`update()` must be fast and
non-blocking**. A busy-wait in one subsystem stalls every other subsystem.

### `SchedulableSubsystem`

The non-template interface the scheduler stores:

```cpp
virtual void update(double dtSeconds) = 0;
virtual const char* getName() const = 0;
```

You do not implement this directly — `Subsystem<StateEnum>` does. The split
exists so subsystems with different state enum types can live in one polymorphic
collection.

## `Subsystem<StateEnum>`

Header-only, because each concrete subsystem supplies its own state enum type.

```cpp
struct Preset {
    const char* name;
    double      positionRaw;   // raw HAL units (motor degrees), per HAL convention
};

struct SubsystemConfig {
    const char*         name;
    control::PIDFConfig pidf;
    std::vector<Preset> presets;
};

template <typename StateEnum>
class Subsystem : public SchedulableSubsystem {
    Subsystem(hal::MotorGroup& motors, const SubsystemConfig& config, StateEnum initialState);

    const char* getName()  const override;
    StateEnum   getState() const;    // safe from any task
    void        update(double dtSeconds) final;
```

### What you override

| Method | When it runs | Typical use |
| --- | --- | --- |
| `onUpdate(double dt)` | Every cycle, lock held | **Required.** Your state machine |
| `onEnter(StateEnum)` | Entering a state | Latch a hold target |
| `onExit(StateEnum)` | Leaving a state | Cleanup, logging |
| `onFault(hal::HealthStatus)` | Motor group reports non-`ok` | Transition to a fault state |

### What you call

```cpp
void transitionTo(StateEnum newState);
void driveToPosition(double targetPositionRaw, double dtSeconds);   // from onUpdate() only
std::optional<double> getPresetPosition(const char* presetName) const;

bool registerFlag(const char* flagName, bool initialValue = false) const;
bool setFlag(const char* flagName, bool value) const;

void lock() const;    // recursive
void unlock() const;
```

`transitionTo()` runs `onExit(old)`, **resets the PIDF controller**, sets the new
state, then runs `onEnter(new)`. A same-state transition is a no-op. Resetting
the controller in between is the point: the new state's target should not inherit
the old one's integral and derivative history.

### Fail-safe behavior

`driveToPosition()` checks health before commanding anything:

* `stalled` or `overTemperature` → **zero voltage and call `onFault()`,
  unconditionally**, whether or not your override reacts to that status.
* `disconnected` → stop only if **every** motor in the group is gone; a partial
  disconnect keeps driving in a degraded state.

The word *unconditionally* is doing real work. Before the review pass, this path
relied entirely on the concrete subsystem's `onFault()` reacting — so a status
the override did not handle could leave stale voltage commanded indefinitely
while the fault was re-detected every cycle.

Output is clamped to ±12000 mV.

### Thread safety

`update()` runs on the scheduler task while command methods (`moveToPreset()`
and friends) may be called from `opcontrol()` or an autonomous routine. The base
class guards `transitionTo()`, `getState()`, and the whole `onUpdate()` cycle
with an internal **recursive** mutex.

Your own cross-task state — a cached target position, say — is **your**
responsibility. Use `lock()`/`unlock()`:

```cpp
void MyArm::moveToPreset(const char* presetName) {
    const auto position = getPresetPosition(presetName);
    if (!position) { return; }

    lock();
    targetPositionRaw_ = *position;
    transitionTo(MyArmState::movingToTarget);   // recursive — safe here
    unlock();
}
```

Recursion matters: `transitionTo()` takes the same lock, and `onFault()` called
from within `onUpdate()` may call it too.

### Automatic telemetry

`update()` records the raw state ordinal to the telemetry bus under the
subsystem's own name, every cycle. No per-subsystem wiring needed. Flags get the
same treatment through `FlagRegistry`.

## `FlagRegistry`

A thread-safe store of named booleans. Subsystems register their flags at
construction and update them from `onUpdate()`; other layers read them by name.
The main consumer today is
[driver control's accel limiter](driver.md#accellimitresolver).

```cpp
static FlagRegistry& instance();

bool registerFlag(const char* name, bool initialValue = false);
bool unregisterFlag(const char* name);
bool setFlag(const char* name, bool value);
bool getFlag(const char* name, bool defaultValue = false) const;
bool isRegistered(const char* name) const;

std::uint8_t getRegisteredCount() const;
const char*  getNameAt(std::uint8_t index) const;
```

Capacity: **16 flags** (`kMaxFlags`). Names must be stable-lifetime strings —
string literals — since the registry stores the pointer, not a copy.

Registration is what makes a **typo detectable**: `setFlag()` returns `false` for
an unregistered name instead of silently doing nothing.

> ⚠️ **No ownership enforcement.** `registerFlag()` rejects a second registration
> of the same name, but if that `false` return is ignored, the second subsystem
> will happily `setFlag()` the name every cycle and stomp the first one's value.
> Give every subsystem's flags a **subsystem-prefixed name**
> (`"lift.isExtended"`, not `"isExtended"`) and check the return value.

Every `setFlag()` also forwards the value to the telemetry bus under the flag's
name, so flags show up in SD logs and the serial stream for free.

## Writing a subsystem

The full pattern, adapted from the demo:

```cpp
// my_lift.hpp
enum class MyLiftState { idle, movingToTarget, holding, faulted };
const char* toString(MyLiftState state);

class MyLift : public Subsystem<MyLiftState> {
public:
    explicit MyLift(hal::MotorGroup& motors);
    void moveToPreset(const char* presetName);   // safe from any task

protected:
    void onUpdate(double dtSeconds) override;
    void onEnter(MyLiftState state) override;
    void onFault(hal::HealthStatus status) override;

private:
    double targetPositionRaw_ = 0.0;   // guarded by lock()/unlock()
};
```

```cpp
// my_lift.cpp — config lives in an anonymous namespace, not in logic
namespace {
const SubsystemConfig kMyLiftConfig{
    .name = "myLift",
    .pidf = { .kP = 15.0, .kD = 1.0, .settleTolerance = 5.0, .settleCycles = 5 },
    .presets = { {"LOW", 0.0}, {"MID", 90.0}, {"HIGH", 180.0} },
};
}

MyLift::MyLift(hal::MotorGroup& motors)
    : Subsystem<MyLiftState>(motors, kMyLiftConfig, MyLiftState::idle) {
    registerFlag("myLift.isExtended", false);
}

void MyLift::onUpdate(double dtSeconds) {
    switch (getState()) {
        case MyLiftState::idle:
            motors_.writeVoltage(0);
            break;
        case MyLiftState::movingToTarget:
            driveToPosition(targetPositionRaw_, dtSeconds);
            if (pidf_.isSettled()) { transitionTo(MyLiftState::holding); }
            break;
        case MyLiftState::holding:
            driveToPosition(targetPositionRaw_, dtSeconds);
            break;
        case MyLiftState::faulted:
            motors_.writeVoltage(0);
            if (motors_.getHealth() == hal::HealthStatus::ok) {
                transitionTo(MyLiftState::idle);
            }
            break;
    }
    setFlag("myLift.isExtended", motors_.getPositionRaw() > kExtendedThreshold);
}

void MyLift::onEnter(MyLiftState state) {
    if (state == MyLiftState::holding) {
        // Latch where we physically are, not where we were driving to.
        targetPositionRaw_ = motors_.getPositionRaw();
    }
}
```

Then in `initialize()`, construct it before `Scheduler::instance().start()` —
and before any `AccelLimitResolver` whose rules reference its flags.

## `demo::ExampleArm` — placeholder

> ⚠️ `ExampleArm` is **not a real mechanism**. It exists to exercise this
> framework end-to-end on a single motor. Delete it once real subsystems exist;
> do not build on it.

It registers `"exampleArm.isExtended"`, exposes `moveToPreset("LOW"/"MID"/"HIGH")`
and a bench-only `simulateFault()`, and is referenced by
`AutonomousContext`, the demo button macro, the driver accel-limit table, and
the dashboard's fault indicator — all of which need revisiting when it goes. See
[checklist § 6](../BUCKET_B_CHECKLIST.md).
