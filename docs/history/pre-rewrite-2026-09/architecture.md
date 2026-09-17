# Architecture

## The shape of the thing

Lightspeed is a **layered** library. Each layer is a namespace under
`lightspeed::`, lives in its own `include/lightspeed/<layer>/` and
`src/lightspeed/<layer>/` directory, and is allowed to depend only on layers
below it in this diagram:

```
                    ┌───────────────────────────┐
                    │  main.cpp (competition)   │
                    │  initialize / autonomous  │
                    │  / opcontrol              │
                    └─────────────┬─────────────┘
                                  │
        ┌────────────┬────────────┼────────────┬─────────────┐
        │            │            │            │             │
   ┌────▼────┐  ┌────▼────┐  ┌────▼─────┐ ┌────▼──────┐ ┌────▼───────┐
   │ driver  │  │  auton  │  │  vision  │ │ telemetry │ │diagnostics │
   └────┬────┘  └────┬────┘  └────┬─────┘ └───────────┘ └────────────┘
        │            │            │
        │       ┌────▼────┐       │
        │       │ motion  │       │
        │       └────┬────┘       │
        │            │            │
        │       ┌────▼────┐◄──────┘
        └──────►│  odom   │
                └────┬────┘
                     │
       ┌─────────────┴──────────┐
  ┌────▼──────┐          ┌──────▼────┐
  │ subsystem │─────────►│  control  │
  └────┬──────┘          └──────┬────┘
       │                        │
       └──────────┬─────────────┘
             ┌────▼────┐
             │   hal   │
             └────┬────┘
                  │
             ┌────▼────┐
             │  PROS   │
             └─────────┘
```

### Dependency rules

* **`hal` talks to PROS. Nothing else does** (except `pros::Task`/`pros::Mutex`,
  which every layer that owns a task or shared state uses directly, and
  `pros::screen`, which only `auton::SelectorGui` and `telemetry::Dashboard`
  draw to).
* **`hal` is unit-agnostic.** It reports raw encoder degrees and raw
  centidegrees. Conversion to inches happens in `odom`; conversion from inches
  per second to motor RPM happens in `motion`. This is deliberate: the same
  `hal::MotorGroup` backs a drivetrain and an arm without knowing what either
  one is.
* **Everything that moves the robot ends at
  `control::DrivetrainVelocityController::setTargetVelocity()`.** Driver
  control, every motion primitive, and every autonomous routine all funnel into
  that one call. Nothing outside `hal::MotorGroup` writes voltage to the drive
  directly.
* **Constants never live in logic.** Every tunable is in a `*_constants.hpp`
  file or a config struct passed at construction. If you find yourself editing a
  number inside a `.cpp`, it belongs in a constants header instead.

## The single-instance rule

`main.cpp` constructs **one** of each hardware and control object in
`initialize()`, holds them in file-scope `std::optional`s, and hands references
to everything that needs them. This is load-bearing, not tidiness:

* The auton selector runs during `initialize()` — before `autonomous()` or
  `opcontrol()` exist — and needs live odometry and motion objects to set a
  starting pose and preview routes.
* If `opcontrol()` built its own `DrivetrainVelocityController` on the same
  ports, two independent PIDF loops would fight over the same physical motors.

So driver control and autonomous share the *same* drivetrain controller
instance, the *same* odometry, and the *same* motion primitives. An autonomous
routine calling `driveStraightDistance.run(24.0)` is exercising exactly the code
path a driver's joystick exercises, one layer up.

## Threading model

Lightspeed runs several PROS tasks concurrently. Rates are chosen per layer, not
globally:

| Task | Owner | Period | Rate | Notes |
| --- | --- | --- | --- | --- |
| Odometry fusion | `odom::OdometryFusion` | 5 ms | ~200 Hz | Fastest loop; pose must lead control |
| Drivetrain velocity | `control::DrivetrainVelocityController` | 10 ms | ~100 Hz | Matches the V5 motor's own update rate — faster gains nothing |
| Subsystem scheduler | `subsystem::Scheduler` | 20 ms | ~50 Hz | One task iterates *all* subsystems |
| Motion primitives | caller's task | 20 ms | ~50 Hz | Blocking; runs on the autonomous task |
| Driver control | `opcontrol()` | 20 ms | ~50 Hz | The PROS-provided task |
| Selector GUI | `auton::SelectorGui` | — | — | Draw + touch poll, `initialize()` onward |
| Dashboard | `telemetry::Dashboard` | 125 ms | ~8 Hz | Driver-control period only |
| SD logger | `telemetry::SdLogger` | 40 ms sample | ~25 Hz | Flushes to card every 10 rows (~400 ms) |
| Serial link | `telemetry::SerialLink` | 100 ms | ~10 Hz | Diagnostic mode only |

Three rules follow from this:

1. **Only one task draws to the screen at a time.** `SelectorGui` owns
   `pros::screen` from `initialize()` until `stop()` is called; `autonomous()`
   and `opcontrol()` both call `stop()` first. `Dashboard` gates itself on
   competition state so it only draws during driver control.
2. **`Subsystem::update()` must not block.** It runs on the shared scheduler
   task, so a busy-wait in one subsystem stalls every other subsystem.
3. **Never call a blocking motion primitive from `opcontrol()`.** That is why
   `driver::ButtonMacroRunner` exists as a non-blocking state machine instead of
   reusing `auton::sequencer`'s blocking `waitUntilSettled()`.

Shared state is guarded explicitly: `OdometryFusion` uses a `pros::MutexVar`,
`Subsystem` a `pros::RecursiveMutex`, `TelemetryBus`/`FlagRegistry` a plain
`pros::Mutex`, and target velocities are `std::atomic<double>`.

## Coordinate and unit conventions

Every layer uses the same conventions. Getting these wrong is the most common
source of confusing behavior, so they are stated once here and referenced
everywhere else.

**Field frame**

* `x` and `y` are in **inches**.
* Heading is in **degrees, clockwise-positive**, wrapped to `[0, 360)`. This
  matches VEXos's native IMU convention.
* At heading 0: local **forward** maps to field **+y**, and local
  **strafe-right** maps to field **+x**.

**Units by layer**

| Quantity | Unit | Where |
| --- | --- | --- |
| Motor encoder position | raw degrees | `hal::MotorGroup::getPositionRaw()` |
| Rotation sensor position | centidegrees | `hal::RotationSensor` |
| Motor velocity | RPM | `hal`, `control`, `driver` |
| Voltage | millivolts, ±12000 | `hal::MotorGroup::writeVoltage()` |
| Distance | inches | `odom`, `motion` |
| Speed | inches/second | `motion` (converted to RPM at the boundary) |
| Angular velocity | degrees/second | `motion` |
| Joystick input | normalized `[-1, 1]` | `driver` |

**Signs**

* A **negative port number** in `hal::config` reverses that motor or sensor,
  matching PROS's own convention.
* `PodConfig::offsetInches` is measured **perpendicular to the pod's rolling
  direction**: a forward pod's offset is its left/right position (positive =
  right), a strafe pod's offset is its forward/back position (positive =
  forward).
* `headingErrorDegrees(from, to)` returns positive for a **clockwise** rotation.

## Naming conventions

| Thing | Style | Example |
| --- | --- | --- |
| Types | `PascalCase` | `DrivetrainVelocityController` |
| Methods / functions | `camelCase` | `setTargetVelocity()` |
| Constants | `k`-prefixed `PascalCase` | `kMaxDriveRpm` |
| Private members | trailing underscore | `config_` |
| Config structs | `<Thing>Config` | `PurePursuitConfig` |
| Constants files | `<layer>_constants.hpp` | `motion_constants.hpp` |

Each file opens with a `\file` block explaining what it is and, more usefully,
*why it is shaped that way*. Those blocks are the primary source for the
per-layer pages in this documentation.

## Layers at a glance

**[hal](layers/hal.md)** — The only file in the project containing a raw port
number is `hal/config.hpp`. Rewiring the robot means editing that one file.

**[control](layers/control.md)** — A generic `PIDFController` with no opinion
about position vs. velocity, a standalone `SlewRateLimiter`, and the
`DrivetrainVelocityController` that composes both on top of two motor groups
with battery-sag compensation and health-based fail-safes.

**[odom](layers/odom.md)** — Config-driven sensor fusion. It supports 0–4
tracking pods in any mix of roles, falls back through a confidence hierarchy as
sensors drop out, and reports a `ConfidenceTier` alongside every pose. It is
deliberately not hardcoded to one pod topology.

**[subsystem](layers/subsystem.md)** — A templated `Subsystem<StateEnum>` base
giving you a state machine, PIDF position control, named presets, fail-safe
fault handling, and automatic registration with a shared scheduler. Plus a
`FlagRegistry` of named booleans that subsystems publish and driver control
reads.

**[driver](layers/driver.md)** — Joystick curve/deadband profiling, three drive
modes, an acceleration limiter driven by subsystem flags, and a non-blocking
button-macro runner.

**[motion](layers/motion.md)** — Trapezoidal/S-curve motion profiling and five
primitives: turn to heading, drive straight, drive to point, move to pose
(boomerang), and pure pursuit path following.

**[auton](layers/auton.md)** — A routine registry and a two-screen touch
selector: tap a start location on a rendered field, then pick a routine from
those valid for that start, previewing its route before confirming.

**[telemetry](layers/telemetry.md)** — A passive named-channel bus that any
layer writes to on its own cycle, consumed independently by an SD CSV logger, a
brain dashboard, and a serial link for laptop-side plotting.

**[vision](layers/vision.md)** — AprilTag detections solved into a candidate
robot pose via a corner-ratio approximation, trust-gated, and blended into
odometry. Not wired into the competition path — see the page for why.

**[diagnostics](layers/diagnostics.md)** — Hold **Y** at boot to enter a service
mode that replaces normal operation with the serial link and vision bench test.
