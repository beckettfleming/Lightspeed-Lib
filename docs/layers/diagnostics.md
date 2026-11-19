---
title: Diagnostics Mode
parent: Layers
nav_order: 10
permalink: /layers/diagnostics/
---

**Namespace:** `lightspeed::diagnostics` · **Header:** `include/lightspeed/diagnostics/diagnostic_mode.hpp`

A startup option that streams live telemetry over USB while the robot otherwise runs normally.

---

## How to enter it

**Hold `Y` on the controller while the program starts, and keep holding it until the controller
rumbles** (or the banner prints in `pros terminal`). The check happens after IMU calibration, so
that's about 3 seconds after the program starts.

The controller must be connected. The check waits up to 3 seconds for it, then samples `Y` for
half a second. The terminal always says what happened:

```
[diagnostics] Y not held at startup -- normal boot, no serial telemetry
[diagnostics] controller not connected -- skipping diagnostic-mode check
```

or, when it works:

```
=== DIAGNOSTIC MODE (Y held at startup) ===
Streaming telemetry over serial as CSV. Driver control and
autonomous still run normally, so you can drive while streaming.
Power-cycle without Y held to stop streaming.
============================================
```

**To leave it: power-cycle without holding Y.**

---

## What it does

```
  initialize()
      |
      +--> build every shared object (drivetrain, odometry, motion, ...)
      |
      +--> runDiagnosticModeIfRequested()
               |
               +-- Y not held? --> return, no-op
               |
               +-- Y held? -----> print the banner, rumble
                                  start SerialLink (10 Hz CSV over USB)
                                  return
      |
      +--> selector GUI, SD logger, dashboard start as normal
```

Normal operation continues, so you can drive the robot (for example on blocks, to tune the
velocity controller) while telemetry streams. Other terminal prints are mixed into the stream;
filter for lines that start with a number when plotting.

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

### AprilTag vision bench test (not currently wired in)

`diagnostic_mode.cpp` no longer calls `vision::runVisionBenchTest()`. To use it, call it
yourself (it never returns). When running, it loops at 10 Hz printing, per cycle:

- Raw corner pixel data for the detected tag
- The computed bearing, distance, and skew
- Gating status — accepted, or rejected with the specific reason
- The resulting correction, if any

Accepted corrections **are applied** to odometry, so you can watch their effect directly against
a physically measured tag placement. That's how you verify the calibration, mount offset, tag
map, and — importantly — the skew formula's sign. See [Vision Layer]({{ site.baseurl }}/layers/vision/).

---

## Why `Y`

`Y` also toggles the claw in driver control. Holding it at startup is harmless, but release it
before driver control starts or the first press may register as a claw toggle.

The button constant is `kBootButton` in
[`diagnostic_mode.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/diagnostics/diagnostic_mode.cpp). Change it there if
you rebind things.

---

## Is it safe to leave in?

Yes. The call sits unconditionally in `initialize()`:

```cpp
diagnostics::runDiagnosticModeIfRequested(*gSerialLink);
```

If Y isn't held, it returns and normal operation is unaffected. It adds about half a second to
startup (up to 3.5 seconds if no controller is connected).

---

## Extending it

To add another harness, edit
[`src/lightspeed/diagnostics/diagnostic_mode.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/diagnostics/diagnostic_mode.cpp).

Options:

- **Start another background task** (like `SerialLink` does) — it runs alongside normal
  operation.
- **Gate on a second button** and branch to a blocking harness such as the vision bench test.

If you build a third or fourth harness, a small menu here — pick with the D-pad, confirm with A
— would be a natural next step. The current single-mode form is deliberately the simplest thing
that solves the "unreachable harness" problem.

---

**See also:** [Tuning Guide]({{ site.baseurl }}/guides/tuning/) · [Vision Layer]({{ site.baseurl }}/layers/vision/) · [Telemetry Layer]({{ site.baseurl }}/layers/telemetry/)
