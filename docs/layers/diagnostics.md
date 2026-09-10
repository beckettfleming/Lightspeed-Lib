---
title: Diagnostics Mode
parent: Layers
nav_order: 10
permalink: /layers/diagnostics/
---

**Namespace:** `lightspeed::diagnostics` · **Header:** `include/lightspeed/diagnostics/diagnostic_mode.hpp`

A boot-time "service mode" that replaces normal robot operation with live telemetry streaming
and the AprilTag vision bench test.

---

## How to enter it

**Hold the `Y` button on the controller while the brain powers on / the program starts.**

That's it. Release it once the banner prints.

```
=== DIAGNOSTIC MODE (Y held at boot) ===
Streaming telemetry over serial + running the AprilTag vision bench
test. This REPLACES normal competition operation for this boot --
power-cycle without Y held to boot normally.
=========================================
```

**To leave it: power-cycle without holding Y.** There is no in-mode exit.

---

## What it does

```
  initialize()
      |
      +--> build every shared object (drivetrain, odometry, motion, ...)
      |
      +--> runDiagnosticModeIfRequested()
               |
               +-- Y not held? --> return immediately, no-op
               |                   normal competition flow continues
               |
               +-- Y held? -----> print the banner
                                  start SerialLink (10 Hz CSV over USB)
                                  run runVisionBenchTest()  <-- never returns
```

Because the vision bench test never returns, `initialize()` never returns either. So the auton
selector GUI, the SD logger, and the dashboard **never start this boot**, and `autonomous()` and
`opcontrol()` never run.

That's the point: it's a **replacement** for normal operation, not something running alongside
it. There's no risk of a diagnostic loop interfering with match behavior, because when
diagnostics are running there is no match behavior.

---

## Why it exists

Two useful test harnesses were built during development and then left completely unreachable:

- **`telemetry::SerialLink`** (Step 8) — fully working, but nothing ever called `start()` on it.
- **`vision::runVisionBenchTest()`** (Step 9) — fully working, but you had to temporarily edit
  `opcontrol()` to call it, then remember to take that call back out.

Both were "wire it in by hand and remember to remove it" harnesses. Diagnostic mode gives them
one real, permanent entry point that costs nothing when not used.

---

## What you get

### Serial telemetry stream

Every telemetry-bus channel, streamed over USB stdout as CSV at 10 Hz — a header line, then one
data line per sample:

```csv
timeMs,odom.pose.x,odom.pose.y,drivetrain.left.targetRpm,drivetrain.left.actualRpm,...
1240,12.5000,36.2000,150.0000,147.3000,...
```

Tail it with `pros terminal` and pipe it into a plotting script. This is the best tool available
for tuning the drivetrain velocity controller — you can watch a step response live instead of
guessing from the robot's behavior. See [Tuning Guide]({{ site.baseurl }}/guides/tuning/).

### AprilTag vision bench test

Loops at 10 Hz printing, per cycle:

- Raw corner pixel data for the detected tag
- The computed bearing, distance, and skew
- Gating status — accepted, or rejected with the specific reason
- The resulting correction, if any

Accepted corrections **are applied** to odometry, so you can watch their effect directly against
a physically measured tag placement. That's how you verify the calibration, mount offset, tag
map, and — importantly — the skew formula's sign. See [Vision Layer]({{ site.baseurl }}/layers/vision/).

---

## Why `Y`

It's the only face button not already bound in `opcontrol()`:

| Button | Normal binding |
|---|---|
| R1 / R2 | Demo arm presets |
| L1 | Intake + demo macro |
| B, Down | Intake |
| **Y** | **free — used here** |

The button constant is `kBootButton` in
[`diagnostic_mode.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/diagnostics/diagnostic_mode.cpp). Change it there if
you rebind things.

---

## Is it safe to leave in?

Yes, and that's the design intent. The call sits unconditionally at the top of `initialize()`:

```cpp
diagnostics::runDiagnosticModeIfRequested(*gVisionCorrector, *gOdometry, *gSerialLink);
```

If Y isn't held, it constructs a `pros::Controller`, reads one button, and returns. Normal
competition operation proceeds completely unaffected. There is no reason to remove it before a
competition.

---

## Extending it

To add another harness, edit
[`src/lightspeed/diagnostics/diagnostic_mode.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/diagnostics/diagnostic_mode.cpp).

Since `runVisionBenchTest()` never returns, anything you add must either run **before** it, or
replace it. Options:

- **Start another background task** before the vision test (like `SerialLink` does) — it keeps
  running alongside.
- **Gate on a second button** and branch to a different blocking harness.

If you build a third or fourth harness, a small menu here — pick with the D-pad, confirm with A
— would be a natural next step. The current single-mode form is deliberately the simplest thing
that solves the "unreachable harness" problem.

---

**See also:** [Tuning Guide]({{ site.baseurl }}/guides/tuning/) · [Vision Layer]({{ site.baseurl }}/layers/vision/) · [Telemetry Layer]({{ site.baseurl }}/layers/telemetry/)
