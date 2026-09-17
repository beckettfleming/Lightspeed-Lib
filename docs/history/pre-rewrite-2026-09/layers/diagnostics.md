# `lightspeed::diagnostics` — service mode

A single boot-gated entry point that consolidates the library's ad-hoc test
harnesses, so they are reachable without editing `main.cpp`.

```cpp
void runDiagnosticModeIfRequested(vision::VisionPoseCorrector& visionCorrector,
                                  odom::OdometryFusion& odometry,
                                  telemetry::SerialLink& serialLink);
```

## How to use it

**Hold `Y` on the master controller while the brain boots.**

* **Not held** — the call returns immediately and does nothing. Normal
  `initialize()` / `autonomous()` / `opcontrol()` operation proceeds completely
  unaffected. This is why `main.cpp` can call it unconditionally near the end of
  `initialize()`.
* **Held** — prints a banner, starts the [serial link](telemetry.md#seriallink),
  and runs the [vision bench test](vision.md#runvisionbenchtest) loop. That loop
  never returns.

## It replaces normal operation

Diagnostic mode is a deliberate **service mode**, not a parallel one.
`initialize()` never returns, so `autonomous()` and `opcontrol()` never run this
boot, and the selector GUI, SD logger, and dashboard never start. Power-cycle
without holding `Y` to boot normally.

That is intentional: the bench test wants exclusive use of stdout and applies
odometry corrections directly, neither of which is something you want happening
alongside a match.

## Why it exists

Two fully-built pieces — the Step 8 serial telemetry link and the Step 9
AprilTag bench test — were unreachable. Neither was wired into `main.cpp`,
because wiring either into the competition path would have compromised it: the
serial link floods stdout, and the vision test writes to odometry using
placeholder calibration.

Leaving them as one-off harnesses a developer had to remember to wire in by hand
meant, in practice, that they would never be run. One button hold makes both
available without touching the verified competition path at all.

## Adding a diagnostic

Anything that needs exclusive hardware access, floods the console, or would be
unsafe during a match belongs here rather than in `opcontrol()`. Add it to
`diagnostic_mode.cpp` — and if you add more than one, add a menu, since the
current implementation runs straight into the vision bench test's infinite loop.
