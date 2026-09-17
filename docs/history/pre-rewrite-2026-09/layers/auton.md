# `lightspeed::auton` — autonomous framework

A routine registry, a two-screen touch selector on the brain, and a couple of
sequencing helpers.

The framework's guiding requirement: an autonomous routine is **not** a separate
implementation of "move the robot." It calls the exact same motion primitives
driver control and bench testing use, through a context struct handed to it.

## `AutonomousContext` and `RoutineFunction`

```cpp
struct AutonomousContext {
    control::DrivetrainVelocityController& drivetrain;
    odom::OdometryFusion&                  odometry;
    motion::TurnToHeading&                 turnToHeading;
    motion::DriveStraightDistance&         driveStraightDistance;
    motion::DriveToPoint&                  driveToPoint;
    motion::PurePursuitController&         purePursuit;
    motion::MoveToPose&                    moveToPose;
    subsystem::demo::ExampleArm&           exampleArm;   // ⚠️ placeholder
};

using RoutineFunction = void (*)(AutonomousContext&);
```

Every member is a reference to an object constructed once in `initialize()` —
the same instances `opcontrol()` drives. Replace `exampleArm` with your real
subsystems when they exist.

## `Routine` and `RoutineRegistry`

```cpp
struct Routine {
    const char* name;
    std::vector<std::size_t> validStartLocationIndices;  // indices into kStartLocations; empty = any
    std::vector<motion::Waypoint> previewPoints;         // drawn on screen 2's preview
    RoutineFunction run;
};

class RoutineRegistry {
    void registerRoutine(const Routine& routine);
    const std::vector<Routine>& getAll() const;
    std::vector<std::size_t> getIndicesForStartLocation(std::size_t startLocationIndex) const;
};
```

`previewPoints` is what the driver sees traced on the field before confirming —
worth populating with the route's actual shape, since it is the fastest way to
catch "I picked the wrong side" before a match.

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
            fireAndForget([&ctx] { ctx.exampleArm.moveToPreset("HIGH"); });
            ctx.driveToPoint.driveToPoint(36.0, 60.0);
            waitUntilSettled([&ctx] {
                return ctx.exampleArm.getState() == subsystem::demo::ExampleArmState::holding;
            }, 2.0);
        },
    };
}
```

Register it in `initialize()`:

```cpp
gRoutineRegistry.registerRoutine(makeLeftSideRushRoutine());
```

`RoutineFunction` is a plain function pointer, so the lambda must be
capture-free — take everything from `ctx`.

## `sequencer.hpp`

Two helpers for coordinating a subsystem action with drivetrain motion.

```cpp
template <typename Callable>
void fireAndForget(Callable&& action);

void waitUntilSettled(const std::function<bool()>& isSettled,
                      double timeoutSeconds,
                      std::uint32_t loopPeriodMs = 20);
```

This is **not** a scheduler. Subsystem command methods are already non-blocking
by construction — the [subsystem scheduler](subsystem.md#scheduler) runs their
`update()` asynchronously. `fireAndForget()` exists to make that explicit and
readable in routine code; `waitUntilSettled()` is the matching join point, and
is generic across subsystems so a routine never needs to know a specific state
enum type.

> `waitUntilSettled()` **blocks the calling task**. That is correct on the
> autonomous task and wrong in `opcontrol()` — see
> [`driver::ButtonMacroRunner`](driver.md#buttonmacrorunner).

## `StartLocation`

```cpp
struct StartLocation {
    const char* name;
    odom::Pose  pose;   // field-relative starting pose
};

inline const std::vector<StartLocation> kStartLocations{ /* two placeholders */ };
```

Tapping one on screen 1 calls `odometry.setPose()` with its pose — which is how
the robot knows where it is at the start of a match.

> ⚠️ The two entries are named "Placeholder Start A/B". Replace with the real
> season's legal starting positions. See
> [checklist § 5](../BUCKET_B_CHECKLIST.md).

## The selector GUI

```cpp
SelectorGui(odom::OdometryFusion& odometry, const RoutineRegistry& registry);

void start();   // begins the background draw + touch-poll task
void stop();    // call before anything else draws to the screen

const Routine* getConfirmedRoutine() const;
const Routine* getRoutineForAutonomous(bool* outUsedFallback = nullptr) const;
```

**Screen 1 — start location.** A schematic top-down field with tappable zones at
each `kStartLocations` entry. Tapping one sets odometry's pose and advances to
screen 2.

**Screen 2 — routine.** A scrollable list of routines *filtered to that start
location*, beside a smaller field preview. Tapping a routine previews its
`previewPoints` on the field; a separate **Confirm** zone locks it in. **Back**
returns to screen 1 and resets screen 2's state.

Nothing is committed until Confirm is tapped — the selection stays live and
re-editable the whole time the GUI is running.

### `getRoutineForAutonomous()` — the no-confirmation fallback

If a routine was confirmed, it returns that. If one was merely *previewed* —
tapped once, Confirm never pressed — it returns **that one anyway**, on the
assumption the driver chose deliberately and forgot to lock it in. It returns
`nullptr` only if nothing was ever selected at all.

`outUsedFallback` reports which path was taken so the caller can log the
distinction rather than it being silent. `autonomous()` prints a warning in the
fallback case.

The reasoning: a robot doing nothing for the entire autonomous period because
someone missed a button tap is a worse failure than running a routine that was
visibly on screen.

### Screen ownership

`SelectorGui` owns `pros::screen` while running. It is the **only** thing that
should draw during that time. Both `autonomous()` and `opcontrol()` call
`stop()` before doing anything else — `opcontrol()` redundantly, to cover the
bench path where it runs without `autonomous()` ever having been called, so the
selector and the [dashboard](telemetry.md#dashboard) never both hold the screen.

## Field rendering

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

The field point `(originXInches, originYInches)` maps to the region's
**bottom-left** corner, scaled uniformly by `pixelsPerInch`. Field +y maps to
screen −y, since screen y grows downward.

One config serves both screens: screen 1 uses it as-is; screen 2's preview pane
derives a scaled copy (same origin, `pixelsPerInch × kPreviewScaleFactor`). The
renderer is a schematic — border and center line drawn with primitives, no bitmap
asset — and does not erase the screen first, so callers control when to clear.

## Layout constants — `screen_layout.hpp`

Every pixel number lives here so nothing in `selector_gui.cpp` hardcodes one.
The V5 brain screen is 480 × 272.

| Constant | Value | What |
| --- | --- | --- |
| `kScreen1FieldRegion` | (125, 24)–(355, 254) | Screen 1's 230×230 field |
| `kStartZoneRadiusPixels` | 10 | Tap radius per start location |
| `kPreviewRegion` | (10, 30)–(160, 180) | Screen 2's preview pane |
| `kPreviewScaleFactor` | 0.65 | Applied to `pixelsPerInch` |
| `kRoutineListX0`/`X1`/`Y0` | 170 / 420 / 30 | Routine row column |
| `kRoutineRowHeight` | 32 | |
| `kMaxVisibleRoutineRows` | 5 | Scroll buttons appear past this |
| `kScrollUpZone` / `kScrollDownZone` | x 430–470 | Reserved column, never overlaps a row |
| `kBackZone` / `kConfirmZone` | (10,236)–(110,266) / (370,236)–(470,266) | |

Plus `insideRegion(x, y, region)`.

## Field constants — `auton_constants.hpp`

```cpp
inline constexpr FieldDimensions kFieldDimensions{ .widthInches = 144.0, .lengthInches = 144.0 };

inline constexpr FieldToScreenConfig kFieldToScreenConfig{
    .originXInches = 0.0,
    .originYInches = 0.0,
    .pixelsPerInch = 1.6,     // 230 px / 144 in, with margin
};
```

> ⚠️ Assumes a standard 12 ft × 12 ft field — confirm against the actual season
> field. If the dimensions change, `pixelsPerInch` must change with them to keep
> the field inside `kScreen1FieldRegion`.

## Demo routines — placeholder

> ⚠️ `demo::makeDemoStraightAndTurnRoutine()` and
> `demo::makeDemoPursuitPathRoutine()` are **not tied to any real season
> strategy**. They exist to validate the registry/GUI/sequencer end-to-end.
> Delete and replace. See [checklist § 5](../BUCKET_B_CHECKLIST.md).

## Telemetry published

| Channel | Type |
| --- | --- |
| `auton.routineName` | text (`"none"` if nothing ran) |
