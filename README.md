# Lightspeed

A from-scratch VEX V5RC code library for team RoboPanthers (robot: Tachyon,
97934U), built on PROS (C++). Layers: `lightspeed::hal` (Step 1),
`lightspeed::control` (Step 2), `lightspeed::odom` (Step 3),
`lightspeed::subsystem` (Step 4), `lightspeed::driver` (Step 5),
`lightspeed::motion` (Step 6), `lightspeed::auton` (Step 7),
`lightspeed::telemetry` (Step 8), `lightspeed::vision` (Step 9).

This document covers the **full review pass** run across all nine steps:
what was verified working end-to-end, what got completed, and what still
needs real robot/season data before competition use (see
[`docs/BUCKET_B_CHECKLIST.md`](docs/BUCKET_B_CHECKLIST.md)).

## Verified working end-to-end

**Build**: the entire project compiles clean under `-Wall -Wextra` on the
ARM GNU Toolchain 14.2.1, zero warnings, zero errors, links successfully.
Verified with a full clean rebuild after every change in this pass, not
just incrementally.

**Cross-layer wiring**, traced explicitly rather than assumed:

- HAL → odometry: `IMESource`/`IMUSource`/`TrackingWheelSource` all read
  through the Step 1 HAL wrappers correctly; `OdometryFusion` reads every
  configured source each fusion cycle.
- HAL → motion control: every motion primitive (`TurnToHeading`,
  `DriveStraightDistance`, `DriveToPoint`, `PurePursuitController`, and the
  new `MoveToPose`) ends by calling
  `DrivetrainVelocityController::setTargetVelocity()` — none write voltage
  directly, confirmed by grepping for `move_voltage`/`writeVoltage` across
  the whole motion layer (zero hits outside `hal::MotorGroup` itself).
- Motion control → driver control: `opcontrol()`'s pipeline (input
  profiling → drive-mode transform → accel-limited slew → velocity
  controller) is the exact same `DrivetrainVelocityController` instance
  autonomous routines use — one shared object, not two competing ones (see
  `main.cpp`'s comment on the shared-globals block, a deliberate Step 7
  design decision that held up under this review).
- Motion control + subsystems → autonomous routines: `AutonomousContext`
  bundles live references to every primitive and to `ExampleArm`; demo
  routines call the exact same functions driver control and bench-testing
  use, per Step 6/7's original requirement.
- Everything → telemetry: HAL health, odometry pose/velocity/confidence,
  subsystem state/flags, and motion target-vs-actual velocity all record
  onto the bus from their own existing update cycles (Step 8's wiring),
  confirmed still intact after this pass's changes.

**Assumptions from later steps' prompts, specifically checked**: Step 7's
"the exact same functions driver control and bench-testing already use" —
held. Step 8's "only one task should own screen drawing at a time" — held
(`SelectorGui`/`Dashboard` never overlap in the wired call flow, see the
Step 8/9 audit below). Step 9's "must not require restructuring the core's
normal arc-based update loop" — held (`applyVisionCorrection()` is purely
additive to `OdometryFusion`).

**What "integration test on hardware" actually means here**: this
environment has no physical V5 brain, motors, or sensors attached. The
closest available approximation was a from-scratch, evidence-based audit —
five parallel deep-dive passes (driver control, motion primitives,
HAL/subsystem safety, auton/telemetry/vision, and a full TODO/placeholder
catalog), each reading every file in its area in full and citing exact
file:line behavior rather than summarizing from memory — plus the ARM
toolchain build as the only ground-truth compiler check available. Driving
under real driver control with the accel-limit condition actually toggling,
running the GUI selector on a real screen, and confirming the dashboard/SD
log against a live match are **not** things this pass could do, and they
remain the first items on the Bucket B checklist's implicit pre-competition
list, not something this pass can claim to have done.

## Part 2 — completed this pass

- **Curvature drive mode** (`lightspeed::driver::curvatureDrive`,
  `DriveMode::curvature`) — didn't exist before; tank/arcade only. Falls
  back to a direct pivot below a small forward-magnitude threshold so the
  robot can still turn in place (pure curvature steering can't, since
  turn's contribution scales toward zero as forward does). Not switched on
  by default (`kDriveMode` stays `arcade`) — which mode feels best is a
  driver-preference call this pass can't make without a driver.
- **Move-to-pose / boomerang primitive**
  (`lightspeed::motion::MoveToPose`) — didn't exist before; `DriveToPoint`
  is a discrete turn-then-drive with no heading target at all. The new
  primitive drives to a full `(x, y, heading)` pose in one continuous
  curved motion via a boomerang carrot point, reusing the same
  curvature-steering math as `PurePursuitController`.
- **Button-macro system** (`lightspeed::driver::ButtonMacroRunner`) —
  flagged "not yet scoped" in Step 5; this pass scoped and built it as a
  non-blocking per-tick state machine (deliberately *not* reusing
  `auton::sequencer`'s blocking `waitUntilSettled`, which would stall the
  whole 50Hz opcontrol loop for a macro's duration). Demonstrated against
  `ExampleArm` via a demo macro bound to L1, since no real subsystem exists
  yet.
- **HAL degraded-capability handling** — `hal::MotorGroup::getHealth()`
  previously collapsed "1 of 3 motors disconnected" and "3 of 3
  disconnected" into the same `disconnected` status with no way to tell
  them apart. Added `getMotorCount()`/`getConnectedMotorCount()`, and wired
  the drivetrain/subsystem fail-safes (see Part 4) to only fully stop a
  side when *every* motor in its group is gone — a partial disconnect now
  keeps driving in a degraded state instead of stopping a side that's
  still partially drivable.
- **S-curve motion profiling** and the **serial/PC live-plotting link**
  were already fully built (Steps 6 and 8 respectively) — verified, not
  rebuilt. The serial link existed but was never wired up anywhere; this
  pass gives it a real entry point via the new diagnostic mode (Part 5)
  rather than leaving it permanently unreachable.

## Part 3 — Bucket A (resolved this pass)

- **AprilTag skew formula**: replaced the original ad hoc
  `acos(shorterEdge/longerEdge)` mapping with a properly derived
  pinhole-projection formula (`tag_pose_solver.cpp`) — same reduction to 0
  at square-on, but now a real closed-form derivation using the
  already-computed distance and known tag width, not an arbitrary curve
  fit. Its *sign* still needs on-hardware verification (which physical
  edge the sensor's corner numbering calls "left"), which is genuinely
  Bucket B, not something a math refinement alone can resolve.
- **No-confirmation fallback**: `autonomous()` previously did nothing for
  the whole period if the driver never tapped Confirm. Added
  `SelectorGui::getRoutineForAutonomous()`: falls back to whatever routine
  was last tentatively previewed (on the assumption the driver picked it
  deliberately and just forgot to lock it in), and only truly does nothing
  if nothing was ever selected at all. Logs which path was taken.
- **FlagRegistry polish**: added `unregisterFlag()` and enumeration
  (`getRegisteredCount()`/`getNameAt()`) for debugging, and documented —
  rather than silently left as a trap — the real remaining gap: there's no
  ownership enforcement, so a second subsystem registering a
  colliding flag name and ignoring the rejected return value can still
  stomp the first subsystem's value. With only one real flag/subsystem in
  the codebase today this is a documented risk for later, not a live bug.
- **Config values reviewed for "needs on-robot tuning" framing** — the vast
  majority already carried clear TODO/reasoning comments from prior steps;
  this pass didn't find undocumented arbitrary guesses worth calling out
  beyond what's already itemized in the Bucket B checklist.

## Part 4 — code quality & safety

- **Real bug fixed**: `DrivetrainVelocityController` read motor health
  every cycle but only ever recorded it for telemetry — a
  stalled/overheating/disconnected drive motor could be commanded
  indefinitely. Now zeroes output (and resets the PIDF integrator + slew
  limiter) on `stalled`/`overTemperature`, or on `disconnected` only once
  every motor in the group is gone.
- **Real bug fixed**: `Subsystem::driveToPosition()`'s fault path only
  zeroed voltage if the concrete subsystem's `onFault()` override happened
  to react to that specific `HealthStatus` (`ExampleArm`'s only handles
  `stalled`) — `overTemperature`/full `disconnected` could leave stale
  voltage commanded indefinitely. The base class now fails safe
  unconditionally, regardless of what (if anything) the override does.
- **Added**: low battery is now a real fail-safe input, not just a
  compensation-math input — below a configurable threshold, battery-sag
  compensation stops *boosting* output further (capped at 1.0x, not cut
  below it — a critically low pack still needs to drive off the field, not
  be stranded).
- **Confirmed already correct, no changes needed**: every voltage output
  path is clamped (traced every `writeVoltage`/`move_voltage` call site in
  the codebase); every blocking motion primitive already had both a settle
  condition and a timeout; the GUI selector's touch handling has no
  crash/hang path on rapid taps, out-of-zone taps, or boundary-pixel taps;
  `SdLogger` already degrades cleanly with no SD card present.
- **Added**: `SdLogger` now checks `fwrite`'s return value — a card pulled
  mid-match now stops logging cleanly instead of silently retrying a dead
  file every cycle for the rest of the run.
- **Naming/namespace consistency**: no drift found across the nine steps'
  layers (`lightspeed::hal`/`control`/`odom`/`subsystem`/`driver`/`motion`/
  `auton`/`telemetry`/`vision`, plus the new `lightspeed::diagnostics`) —
  all consistently PascalCase types, camelCase methods, `k`-prefixed
  constants, config-struct-over-magic-number.

## Part 5 — consolidated diagnostic mode

Two previously scattered, unwired-anywhere test entry points — the Step 8
serial telemetry link and the Step 9 AprilTag vision bench test — are now
one selectable mode (`lightspeed::diagnostics::runDiagnosticModeIfRequested`,
`include/lightspeed/diagnostics/diagnostic_mode.hpp`), gated behind holding
Y at boot. If not held, it's a no-op and normal competition operation
proceeds completely unaffected. If held, it's a deliberate "service mode"
that replaces normal operation for that boot (never returns) rather than
running alongside it.

## What still needs real data before competition use

See **[`docs/BUCKET_B_CHECKLIST.md`](docs/BUCKET_B_CHECKLIST.md)** for the
complete, organized list: port wiring, odometry geometry, every control
gain, AprilTag calibration/mounting/tag-map, season field/start
locations/auton strategy, and the real subsystem mechanisms that
`ExampleArm` currently stands in for. Nothing on that list was guessed at
or faked to "look done" — every entry is a clearly-marked placeholder with
a pointer to exactly where it lives.
