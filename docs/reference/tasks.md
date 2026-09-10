---
title: Task and Timing Reference
parent: Reference
nav_order: 3
permalink: /reference/tasks/
---

Lightspeed runs several PROS tasks concurrently. This page collects every rate in one place and
explains why each was chosen.

---

## Every loop

| Loop | Period | Rate | Owner | Constant |
|---|---|---|---|---|
| Odometry fusion | 5 ms | **200 Hz** | `OdometryFusion` | `kLoopPeriodMs` in `odometry_fusion.cpp` |
| Drivetrain velocity control | 10 ms | **100 Hz** | `DrivetrainVelocityController` | `kDrivetrainVelocityConfig.loopPeriodMs` |
| `opcontrol()` | 10 ms | **100 Hz** | PROS entry point | `kLoopPeriodMs` in `main.cpp` |
| Subsystem scheduler | 20 ms | **50 Hz** | `Scheduler` | `kLoopPeriodMs` in `scheduler.hpp` |
| Motion primitives | 20 ms | **50 Hz** | each primitive | `loopPeriodMs` in each config |
| `waitUntilSettled()` polling | 20 ms | 50 Hz | `auton::sequencer` | default parameter |
| SD logger sample | 40 ms | **25 Hz** | `SdLogger` | `kSdLoggerSampleIntervalMs` |
| SD logger flush | ~400 ms | ~2.5 Hz | `SdLogger` | `kSdLoggerRowsPerFlush` (10) |
| Serial link | 100 ms | **10 Hz** | `SerialLink` | `kSerialLinkLoopPeriodMs` |
| Vision bench test | 100 ms | **10 Hz** | `runVisionBenchTest` | `kVisionBenchTestLoopPeriodMs` |
| Dashboard redraw | 125 ms | **8 Hz** | `Dashboard` | `kDashboardLoopPeriodMs` |
| `opcontrol()` heartbeat print | 1000 ms | 1 Hz | `main.cpp` | `kStatusIntervalMs` |
| Motion primitive progress print | 250 ms | 4 Hz | each primitive | `kPrintIntervalMs` |

---

## Why each rate

### 200 Hz — odometry fusion

The fastest loop, because integration error compounds. Sampling position frequently keeps each
cycle's arc small, which keeps the arc-to-chord approximation and the midpoint-heading rotation
accurate. Everything else reads a *snapshot* of the pose, so nothing downstream needs to be this
fast.

### 100 Hz — drivetrain velocity control

**Matched to the V5 motor's own internal update rate.** Running faster gains nothing — the motor
won't act on commands more often than that — and just burns CPU.

> Don't raise this. It's the one rate in the project with a hard hardware justification.

### 100 Hz — `opcontrol()`

Matches the velocity controller underneath. Also the rate at which `dtSeconds` is computed for
the driver-side slew limiters, so the acceleration cap is accurate.

### 50 Hz — subsystem scheduler

Mechanisms are physically slower than a drivetrain and don't need 100 Hz position control.
50 Hz keeps CPU load down while staying responsive.

**All subsystems share this one task**, so a slow `update()` in one delays every other. Keep them
fast and non-blocking.

### 50 Hz — motion primitives

Enough for odometry-feedback motion. The 100 Hz velocity controller underneath does the fast
work; the primitive only has to keep updating its target.

### 25 Hz — SD logging

A compromise between log resolution and file size. At 25 Hz, a two-minute match produces roughly
3000 rows — plenty to reconstruct what happened without an unwieldy file.

### 8 Hz — dashboard

Fast enough to look live to a human; slow enough that screen drawing (which is relatively
expensive) doesn't compete with control loops.

---

## Task priorities

Most tasks run at PROS's default priority. The exception is **`SdLogger`, which runs
low-priority** — file I/O should never delay a control loop.

---

## How the loops keep time

Every periodic loop uses `pros::Task::delay_until(&previousTime, periodMs)` rather than
`pros::delay(periodMs)`.

The difference matters:

| | `delay()` | `delay_until()` |
|---|---|---|
| Waits | a fixed duration **after the work** | until an **absolute** timestamp |
| Effective period | `period + workTime` — **drifts** | exactly `period` |

With `delay()`, a loop doing 3 ms of work with a 5 ms delay actually runs every 8 ms — and the
`dtSeconds` used in the PID math would be wrong. `delay_until()` fixes the period regardless of
how long the work took.

`opcontrol()` uses a plain `pros::delay()`, which is conventional for the PROS driver loop.

---

## Task lifecycle

### Started in `initialize()`

| Task | When |
|---|---|
| Odometry fusion | At the end of the `OdometryFusion` constructor |
| Drivetrain control | At the end of the `DrivetrainVelocityController` constructor |
| Subsystem scheduler | Explicitly, via `Scheduler::instance().start()` |
| Selector GUI | Explicitly, via `SelectorGui::start()` |
| SD logger | Explicitly, via `SdLogger::start()` |
| Dashboard | Explicitly, via `Dashboard::start()` |

### Started only in diagnostic mode

| Task | When |
|---|---|
| Serial link | `SerialLink::start()` inside `runDiagnosticModeIfRequested()` |

### Stopped

| Task | When |
|---|---|
| Selector GUI | At the top of `autonomous()` and `opcontrol()` |
| Everything else | Runs for the program's lifetime |

---

## Two construction-order patterns worth knowing

Both exist to prevent a background task from observing half-built state.

**`DrivetrainVelocityController`** declares its `pros::Task` member **last**:

```cpp
    std::atomic<double> rightTargetRpm_{0.0};
    // Must be declared last: its initializer starts the control task, which
    // reads every other member above and must see them fully constructed.
    pros::Task task_;
```

C++ initializes members in declaration order, so by the time the task starts, everything it
reads already exists.

**`OdometryFusion`** uses a `std::optional<pros::Task>` and `emplace()`s it at the **end of the
constructor body**, after pod partitioning finishes:

```cpp
    // Deferred-start: everything the task reads above must be fully set up.
    task_.emplace([this] { fusionLoop(); }, "lightspeed_odometry_fusion");
```

Same goal, but it also covers work done in the constructor *body*, not just member init.

If you add a class that owns a task, copy one of these patterns.

---

## Blocking calls, and where they're allowed

| Call | Blocks | Safe in |
|---|---|---|
| `Imu::calibrate(true)` | ~2–3 s | `initialize()` only |
| `TurnToHeading::run()` etc. | until settled or timeout | autonomous task, bench harness |
| `auton::waitUntilSettled()` | until settled or timeout | autonomous task |
| `SdLogger` flush | brief file I/O | its own low-priority task |
| `runVisionBenchTest()` | **forever** | diagnostic mode only |

**Never call a blocking motion primitive or `waitUntilSettled()` from `opcontrol()`.** It would
stall the 100 Hz loop — including the drivetrain's own `setTargetVelocity()` calls — and the
driver would lose control. Use `driver::ButtonMacroRunner` instead, which advances one step per
tick. See [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/).

---

## Screen ownership timeline

```
  boot ----> initialize() ----> [match] ----> autonomous() ----> opcontrol()
              |                                  |                  |
              +-- SelectorGui draws              +-- stop()         +-- stop() (idempotent)
                                                     nothing draws      Dashboard draws
```

`Dashboard` also gates itself on `!is_disabled() && !is_autonomous()`, so even if it were started
early it would stay quiet until driver control.

---

**See also:** [Architecture]({{ site.baseurl }}/architecture/) · [Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/)
