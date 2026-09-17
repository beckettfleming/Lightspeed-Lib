# `lightspeed::auton` — autonomous routines and the selector

This layer has three parts:

* a **list of routines** you can pick from,
* a **touchscreen menu** on the brain for picking the start location and routine,
  and
* two small **helpers** for coordinating mechanisms with driving.

The main idea: **a routine doesn't have its own way of moving the robot.** It uses
the same movement commands and mechanisms as everything else, handed to it in a
context object.

---

## `AutonomousContext`

Everything a routine can use:

```cpp
struct AutonomousContext {
    control::DrivetrainVelocityController& drivetrain;
    odom::OdometryFusion&                  odometry;
    motion::TurnToHeading&                 turnToHeading;
    motion::DriveStraightDistance&         driveStraightDistance;
    motion::DriveToPoint&                  driveToPoint;
    motion::PurePursuitController&         purePursuit;
    motion::MoveToPose&                    moveToPose;
    subsystem::demo::ExampleArm&           exampleArm;   // ⚠️ demo only
};

using RoutineFunction = void (*)(AutonomousContext&);
```

Each item is a reference to the one shared object created in `initialize()`.
When you add real mechanisms, add them here (and fill them in inside
`autonomous()` in `main.cpp`), and remove `exampleArm`.

---

## Routines

```cpp
struct Routine {
    const char* name;                                    // shown on screen
    std::vector<std::size_t> validStartLocationIndices;  // which starts it works from; empty = any
    std::vector<motion::Waypoint> previewPoints;         // path drawn on the preview
    RoutineFunction run;                                 // the code to run
};

class RoutineRegistry {
    void registerRoutine(const Routine& routine);
    const std::vector<Routine>& getAll() const;
    std::vector<std::size_t> getIndicesForStartLocation(std::size_t startLocationIndex) const;
};
```

* `validStartLocationIndices` refers to positions in `kStartLocations` (0 = first
  start location). Leave it empty if the routine works from anywhere.
* **Fill in `previewPoints`** with the rough path. The driver sees it on screen
  before confirming — the quickest way to catch "wrong side of the field."
* **Register all routines before the selector starts** (before
  `gSelectorGui->start()` in `initialize()`).

### Writing a routine

```cpp
Routine makeLeftSideRushRoutine() {
    return Routine{
        .name = "Left Rush",
        .validStartLocationIndices = {0},
        .previewPoints = { {12, 12}, {12, 48}, {36, 60} },
        .run = [](AutonomousContext& ctx) {
            ctx.driveStraightDistance.run(36.0);
            ctx.turnToHeading.run(90.0);
            fireAndForget([&ctx] { ctx.exampleArm.moveToPreset("HIGH"); });   // arm moves while driving
            ctx.driveToPoint.driveToPoint(36.0, 60.0);
            waitUntilSettled([&ctx] {
                return ctx.exampleArm.getState() == subsystem::demo::ExampleArmState::holding;
            }, 2.0);   // wait up to 2 s for the arm
        },
    };
}
```

Then register it in `initialize()`:

```cpp
gRoutineRegistry.registerRoutine(makeLeftSideRushRoutine());
```

**The outer lambda can't capture anything** — `[]` must be empty, because
`RoutineFunction` is a plain function pointer. Get everything from `ctx`.
(Lambdas *inside* the routine, like the ones passed to `fireAndForget`, can
capture.)

---

## Helpers — `sequencer.hpp`

```cpp
template <typename Callable>
void fireAndForget(Callable&& action);

void waitUntilSettled(const std::function<bool()>& isSettled,
                      double timeoutSeconds,
                      std::uint32_t loopPeriodMs = 20);
```

* **`fireAndForget(action)`** just runs `action`. Mechanism commands like
  `moveToPreset()` already return immediately and do their work in the
  background, so this mostly makes routine code easier to read: "start this and
  keep going."
* **`waitUntilSettled(check, timeout)`** waits until `check` returns true, or the
  timeout passes. Use it to wait for a mechanism.

> `waitUntilSettled()` **blocks**. Fine in autonomous; never use it in
> `opcontrol()` — use a [button macro](driver.md#button-macros) there.

---

## Start locations — `start_location.hpp`

```cpp
struct StartLocation {
    const char* name;
    odom::Pose  pose;   // where the robot starts on the field
};

inline const std::vector<StartLocation> kStartLocations{ /* two placeholders */ };
```

Tapping a start location on screen tells odometry the robot is at that pose.
That's how the robot knows where it is when autonomous begins.

> ⚠️ The two entries are "Placeholder Start A" (12, 12, facing 0°) and
> "Placeholder Start B" (132, 12, facing 180°). Replace them with the season's real
> starting spots. See [placeholders § 5](../placeholders.md#5-this-seasons-game).

---

## The selector screen

```cpp
SelectorGui(odom::OdometryFusion& odometry, const RoutineRegistry& registry);

void start();   // start drawing and watching for taps
void stop();    // call before anything else draws on the screen

const Routine* getConfirmedRoutine() const;
const Routine* getRoutineForAutonomous(bool* outUsedFallback = nullptr) const;
```

### Using it

**Screen 1 — where are you starting?** A simple field drawing with a yellow dot
for each start location. Tap one. Odometry is set to that position and screen 2
opens.

**Screen 2 — which routine?** A list of routines that work from that start
(scroll buttons appear if there are more than 5), next to a small field preview.

* Tap a routine to highlight it (yellow) and see its path.
* Tap **Confirm** to lock it in (green).
* Tap **Back** to return to screen 1. This clears your routine choice.

You can change your mind any time until autonomous starts.

> **Place the robot before tapping a start location.** The position is set at the
> moment you tap; if the robot is moved afterwards, odometry follows the movement
> ([code review #9](../code-review.md#9-the-starting-pose-is-set-when-you-tap-not-when-the-match-starts)).
>
> **You need a competition switch or field control to use it.** Without one, PROS
> goes straight to driver control, which closes the selector.
>
> ⚠️ Quick taps might be missed — tap firmly
> ([code review #10](../code-review.md#10-selector-taps-may-be-missed)).

### If nobody tapped Confirm

`getRoutineForAutonomous()` returns:

1. the **confirmed** routine, if there is one;
2. otherwise, the **highlighted** routine (tapped but not confirmed), and sets
   `outUsedFallback` to true;
3. otherwise, `nullptr`.

`autonomous()` prints a warning when it uses the highlighted routine, and does
nothing if there's no routine at all. The idea: running the routine that was
clearly on screen is better than sitting still for 15 seconds because someone
forgot a tap.

### Sharing the screen

While running, the selector is the only thing that draws on the screen.
`autonomous()` and `opcontrol()` both call `stop()` first. (`opcontrol()` does it
too in case autonomous never ran.) That way the selector and the
[dashboard](telemetry.md#dashboard) never draw at the same time.

---

## Drawing the field

```cpp
struct ScreenPoint  { std::int16_t x, y; };
struct ScreenRegion { std::int16_t x0, y0, x1, y1; };

struct FieldToScreenConfig { double originXInches, originYInches, pixelsPerInch; };
struct FieldDimensions     { double widthInches, lengthInches; };

ScreenPoint fieldToScreen(double fieldXInches, double fieldYInches,
                          const FieldToScreenConfig&, const ScreenRegion&);

void renderField(const FieldToScreenConfig&, const FieldDimensions&, const ScreenRegion&);
void renderRoute(const FieldToScreenConfig&, const ScreenRegion&,
                 const std::vector<motion::Waypoint>& points);
```

* Field point (`originXInches`, `originYInches`) lands at the **bottom-left**
  corner of the screen region. Field +y points **up** on screen.
* `renderField` draws a border and a center line — no images. It doesn't clear
  the screen first.
* Screen 2's preview uses the same settings, scaled down by `kPreviewScaleFactor`.

### Screen layout — `screen_layout.hpp`

All pixel positions are here, so the selector code has no raw pixel numbers. The
brain screen is 480 × 272 pixels.

| Constant | Value | What |
| --- | --- | --- |
| `kScreen1FieldRegion` | (125, 24)–(355, 254) | Screen 1's field, 230 × 230 px |
| `kStartZoneRadiusPixels` | 10 | Size of each tappable start dot |
| `kPreviewRegion` | (10, 30)–(160, 180) | Screen 2's preview |
| `kPreviewScaleFactor` | 0.65 | Preview size compared to screen 1 |
| `kRoutineListX0` / `X1` / `Y0` | 170 / 420 / 30 | Routine list position |
| `kRoutineRowHeight` | 32 | Height of each row |
| `kMaxVisibleRoutineRows` | 5 | Scroll buttons appear after this |
| `kScrollUpZone` / `kScrollDownZone` | x 430–470 | Scroll buttons |
| `kBackZone` | (10, 236)–(110, 266) | Back button |
| `kConfirmZone` | (370, 236)–(470, 266) | Confirm button |

---

## Field settings — `auton_constants.hpp`

```cpp
inline constexpr FieldDimensions kFieldDimensions{ .widthInches = 144.0, .lengthInches = 144.0 };

inline constexpr FieldToScreenConfig kFieldToScreenConfig{
    .originXInches = 0.0,
    .originYInches = 0.0,
    .pixelsPerInch = 1.6,   // fits 144 in into 230 px
};
```

> ⚠️ Assumes a 12 × 12 ft field. If the size changes, change `pixelsPerInch` too
> so the field still fits in `kScreen1FieldRegion`.

---

## Demo routines

> ⚠️ `demo::makeDemoStraightAndTurnRoutine()` and `demo::makeDemoPursuitPathRoutine()`
> are **not real strategy**. They exist to test the menu and helpers. Replace them.

* **Demo: Straight + Turn** — starts raising the arm, drives 24 in forward, waits
  up to 2 s for the arm, then turns to face 90°.
* **Demo: Pursuit Path** — follows a short 3-point path starting from wherever
  the robot is.

Both are allowed from any start location. Their previews are drawn from Start A,
even if Start B is chosen.

---

## Telemetry channels

| Channel | Type |
| --- | --- |
| `auton.routineName` | text (`"none"` if nothing ran) |
