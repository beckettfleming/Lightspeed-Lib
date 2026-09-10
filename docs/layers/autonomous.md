---
title: Autonomous
parent: Layers
nav_order: 7
permalink: /layers/autonomous/
---

**Namespace:** `lightspeed::auton` · **Headers:** `include/lightspeed/auton/`

Everything for the 15-second autonomous period: a routine registry, a two-screen touchscreen
selector, sequencing helpers, and the field renderer that draws route previews.

---

## How a routine gets run

```
  initialize()
      |
      +--> register routines with RoutineRegistry
      +--> SelectorGui::start()  (owns the brain screen)
                 |
                 v
      +------------------------------+     +------------------------------+
      | SCREEN 1                     |     | SCREEN 2                     |
      | Pick a start location        | --> | Pick a routine + preview     |
      | (also calls odometry         |     | (filtered to that start)     |
      |  .setPose() for that spot)   | <-- | Back / Confirm               |
      +------------------------------+     +------------------------------+
                                                    |
                                              Confirm tapped
                                                    v
  autonomous()
      |
      +--> SelectorGui::stop()   (release the screen)
      +--> getRoutineForAutonomous()
      |         confirmed?  -> run it
      |         previewed?  -> run it, log a WARNING (fallback)
      |         neither?    -> log and do nothing
      +--> build AutonomousContext (all the shared objects)
      +--> routine->run(ctx)
      +--> print the final pose
```

---

## `AutonomousContext`

The bundle of shared objects a routine gets. **These are the exact same instances driver control
uses** — not a separate copy.

```cpp
struct AutonomousContext {
    control::DrivetrainVelocityController& drivetrain;
    odom::OdometryFusion& odometry;
    motion::TurnToHeading& turnToHeading;
    motion::DriveStraightDistance& driveStraightDistance;
    motion::DriveToPoint& driveToPoint;
    motion::PurePursuitController& purePursuit;
    motion::MoveToPose& moveToPose;
    subsystem::demo::ExampleArm& exampleArm;
};

using RoutineFunction = void (*)(AutonomousContext&);
```

A routine **calls into these**. It is never a separate implementation of "move the robot".

> When you add real subsystems, add them here as references and construct them in
> `initialize()` — that's how routines reach them.

---

## `Routine` and `RoutineRegistry`

```cpp
struct Routine {
    const char* name;                                   // shown in the selector
    std::vector<std::size_t> validStartLocationIndices; // empty = valid from any start
    std::vector<motion::Waypoint> previewPoints;        // drawn on screen 2
    RoutineFunction run;
};
```

```cpp
auton::RoutineRegistry registry;
registry.registerRoutine(makeMyRoutine());

const auto& all = registry.getAll();
auto indices = registry.getIndicesForStartLocation(0);   // filtered
```

`validStartLocationIndices` is what makes the selector's filtering work: pick a start location
on screen 1, and screen 2 only offers routines legal from there. An empty vector means "valid
from anywhere".

`previewPoints` is purely illustrative — a rough sketch of the route drawn over the mini field
so the driver can confirm at a glance they picked the right one. It does **not** have to match
what the routine actually does.

---

## Start locations

[`include/lightspeed/auton/start_location.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/start_location.hpp)

```cpp
struct StartLocation {
    const char* name;
    odom::Pose pose;    // field-relative starting pose
};

inline const std::vector<StartLocation> kStartLocations{
    StartLocation{.name = "Placeholder Start A",
                  .pose = {.xInches = 12.0,  .yInches = 12.0, .headingDegrees = 0.0}},
    StartLocation{.name = "Placeholder Start B",
                  .pose = {.xInches = 132.0, .yInches = 12.0, .headingDegrees = 180.0}},
};
```

> 🚧 **Both are placeholders.** Replace them with the real season's legal starting positions.

Tapping a start location on screen 1 immediately calls `odometry.setPose()` with that pose —
which is how the robot knows where it is at the start of a match. **This is the single most
important thing the selector does.** If the driver forgets to pick a start location, odometry
begins at `(0, 0, 0)` and every field-coordinate routine drives to the wrong place.

---

## `SelectorGui`

A two-screen touch GUI on the 480×272 brain screen, running its own background task.

### Screen 1 — start location picker

Draws a schematic top-down field with a circular tap zone at each start location. Tapping one:

1. Records the selection
2. Calls `odometry.setPose()` with that location's pose
3. Advances to screen 2

### Screen 2 — routine picker

- A small field preview on the left with the selected routine's `previewPoints` overlaid
- A scrollable list of routines, filtered to the chosen start location (5 visible rows, with
  scroll buttons appearing only when the list overflows)
- **Back** returns to screen 1 and resets screen 2's state
- **Confirm** locks in the routine

### Nothing is locked in until Confirm

The GUI stays live and fully re-editable the whole time it's running. Tapping a routine only
sets it as *tentative* — Back always works, and you can change your mind right up until Confirm.

### Interface

```cpp
auton::SelectorGui gui(odometry, registry);
gui.start();
gui.stop();     // call before ANYTHING else draws to the screen

const auton::Routine* r = gui.getConfirmedRoutine();   // nullptr if never confirmed

bool usedFallback = false;
const auton::Routine* r2 = gui.getRoutineForAutonomous(&usedFallback);
```

### The no-confirmation fallback

`getRoutineForAutonomous()` exists because of a real failure mode: previously, if the driver
never tapped Confirm, `autonomous()` did **nothing for the entire period**.

Now:

| Situation | Result |
|---|---|
| A routine was confirmed | Run it |
| A routine was previewed but never confirmed | **Run it anyway**, set `usedFallback = true` |
| Nothing was ever selected | Return `nullptr`; log and do nothing |

The assumption is that a driver who tapped a routine picked it deliberately and just forgot to
lock it in before the timer started. `autonomous()` logs loudly which path it took:

```
[autonomous] running confirmed routine: Demo: Straight + Turn
[autonomous] WARNING: no routine was confirmed -- running the last previewed routine as a fallback: ...
[autonomous] no routine confirmed or previewed via the GUI selector -- nothing to run
```

### Screen ownership

`SelectorGui` is the **only** task allowed to draw while it's running. Both `autonomous()` and
`opcontrol()` call `stop()` at their top before anything else touches the screen. See
[Architecture]({{ site.baseurl }}/architecture/#who-owns-the-screen).

The touch handling has been checked for crash and hang paths on rapid taps, out-of-zone taps,
and boundary-pixel taps.

---

## Field rendering

```cpp
void renderField(const FieldToScreenConfig& config, const FieldDimensions& field,
                 const ScreenRegion& region);
void renderRoute(const FieldToScreenConfig& config, const ScreenRegion& region,
                 const std::vector<motion::Waypoint>& points);
```

Schematic drawing with simple shapes — no bitmap asset needed. `renderField()` draws a border
and center line; `renderRoute()` draws dots at each waypoint connected by lines.

Neither erases the screen first; callers control when to erase.

### Field-to-screen mapping

```cpp
ScreenPoint fieldToScreen(double fieldXInches, double fieldYInches,
                          const FieldToScreenConfig& config, const ScreenRegion& region);
```

Field `(originXInches, originYInches)` maps to the region's **bottom-left** corner, scaled
uniformly by `pixelsPerInch`. **Field +y maps to screen −y** (upward), because screen y grows
downward.

One config serves both screens: screen 1 draws at full size; screen 2's preview pane derives a
scaled copy (same origin, `pixelsPerInch × 0.65`).

Layout constants — every pixel coordinate in the GUI — live in
[`screen_layout.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/screen_layout.hpp). Nothing in
`selector_gui.cpp` hardcodes a raw pixel number.

---

## Sequencer helpers

```cpp
template <typename Callable> void fireAndForget(Callable&& action);
void waitUntilSettled(const std::function<bool()>& isSettled,
                      double timeoutSeconds, std::uint32_t loopPeriodMs = 20);
```

**Not a scheduler.** Subsystem command methods are already non-blocking by construction (the
subsystem scheduler runs their `update()` asynchronously). These two functions exist to make
that pattern *explicit and readable* in routine code, and to give routines a generic join point
that doesn't need to know a specific subsystem's state enum.

```cpp
// Start the arm moving, but don't wait — drive at the same time
fireAndForget([&] { ctx.exampleArm.moveToPreset("HIGH"); });

ctx.driveStraightDistance.run(24.0);    // happens concurrently with the arm

// Now join: wait for the arm before turning (2 second timeout)
waitUntilSettled([&] {
    return ctx.exampleArm.getState() == subsystem::demo::ExampleArmState::holding;
}, 2.0);

ctx.turnToHeading.run(90.0);
```

`waitUntilSettled()` **blocks the calling task**, which is correct for the autonomous task and
wrong for `opcontrol()`. If you need this behavior during driver control, use
`driver::ButtonMacroRunner` instead — see [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/).

It returns silently on timeout rather than throwing or asserting, so a mechanism that jams can't
hang the whole routine.

---

## Writing a routine

{% raw %}
```cpp
// src/lightspeed/auton/my_routines.cpp
#include "lightspeed/auton/my_routines.hpp"
#include "lightspeed/auton/sequencer.hpp"

namespace lightspeed::auton {
namespace {

void runScoreAndPark(AutonomousContext& ctx) {
    // Start the lift while driving — no waiting
    fireAndForget([&] { ctx.lift.moveToPreset("HIGH"); });

    ctx.driveStraightDistance.run(24.0);

    // Join before doing something that depends on the lift
    waitUntilSettled([&] { return ctx.lift.getState() == LiftState::holding; }, 2.0);

    ctx.turnToHeading.run(90.0);
    ctx.moveToPose.run(48.0, 72.0, 180.0);
}

}  // namespace

Routine makeScoreAndParkRoutine() {
    return Routine{
        .name = "Score + Park",
        .validStartLocationIndices = {0},           // only from start location 0
        .previewPoints = {{12,12}, {12,36}, {48,72}},
        .run = runScoreAndPark,
    };
}

}  // namespace lightspeed::auton
```
{% endraw %}

Then register it in `initialize()`:

```cpp
gRoutineRegistry.registerRoutine(auton::makeScoreAndParkRoutine());
```

### Tips

- **Keep routines short and composed.** They should read as a list of primitive calls.
- **Use absolute field coordinates** where you can — they're more robust than relative moves,
  since odometry is already tracking the pose the start location set.
- **Give every primitive a realistic timeout.** A routine that hangs on step 2 scores nothing.
- **Test each primitive independently first.** Don't debug a five-step routine when one step is
  mistuned.
- **Watch the console.** Every primitive prints progress; the final pose is printed at the end.

---

## The demo routines

> 🚧 **`Demo: Straight + Turn` and `Demo: Pursuit Path` are placeholders with no relationship to
> any real game strategy.** They exist to validate the registry, GUI, and sequencer end to end.
> Delete them once real routines exist.

They're worth reading as examples, though —
[`src/lightspeed/auton/demo_routines.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/auton/demo_routines.cpp) —
particularly the first, which demonstrates the fire-and-forget/join pattern properly: the arm
raises *while* the robot drives, and the routine only waits for it before the turn.

---

## Configuration

[`include/lightspeed/auton/auton_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/auton_constants.hpp)

```cpp
inline constexpr FieldDimensions kFieldDimensions{
    .widthInches = 144.0, .lengthInches = 144.0     // TODO: confirm against the season field
};

inline constexpr FieldToScreenConfig kFieldToScreenConfig{
    .originXInches = 0.0,
    .originYInches = 0.0,
    .pixelsPerInch = 1.6,      // 230px region / 144in field
};
```

---

**Next:** [Telemetry Layer]({{ site.baseurl }}/layers/telemetry/) — recording what happened.
