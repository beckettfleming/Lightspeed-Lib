---
title: Telemetry
parent: Layers
nav_order: 8
permalink: /layers/telemetry/
---

**Namespace:** `lightspeed::telemetry` · **Headers:** `include/lightspeed/telemetry/`

One shared data bus that every layer publishes to, and three independent consumers: an SD-card
CSV logger, an on-screen dashboard, and a USB serial stream for live plotting.

---

## The shape of it

```
   HAL / control / odom / subsystem / auton
        |  each calls record(name, value) from its OWN update cycle
        v
   +----------------------+
   |    TelemetryBus      |   singleton, 32 channels max
   |  (passive snapshot)  |   no polling loop of its own
   +----------+-----------+
              | getSnapshot()  — each consumer polls at its own rate
      +-------+--------+-----------------+
      v                v                 v
  +--------+     +-----------+     +------------+
  |SdLogger|     | Dashboard |     | SerialLink |
  | 25 Hz  |     |   8 Hz    |     |   10 Hz    |
  | CSV to |     | brain     |     | USB CSV    |
  | /usd   |     | screen    |     | (diag mode)|
  +--------+     +-----------+     +------------+
```

**The bus is passive.** It doesn't own a polling loop and doesn't decide how often anything is
sampled. Producers push on their own schedule; consumers pull on theirs. This means a
timing-sensitive loop like the 200 Hz odometry fusion never blocks on I/O — `record()` is just
an in-memory array write under a short mutex.

---

## `TelemetryBus`

```cpp
auto& bus = telemetry::TelemetryBus::instance();

bus.record("odom.pose.x", 12.5);        // double
bus.record("hal.imu.healthy", true);    // bool
bus.record("drivetrain.left.connectedMotors", 2);   // std::int32_t
bus.record("auton.routineName", "Score + Park");    // const char*

telemetry::TelemetryBus::Snapshot snap = bus.getSnapshot();
for (std::uint8_t i = 0; i < snap.count; ++i) {
    const auto& ch = snap.channels[i];   // .name, .type, .number/.integer/.boolean/.text
}
```

- **32 channels max** (`kMaxChannels`)
- Channels register **on first use**, in call order, and that order is **stable thereafter** —
  which is what lets the CSV header stay valid for the whole run
- `getSnapshot()` copies a fixed-size array — cheap, no allocation, safe from any task

### Channel names must be string literals

The bus stores the **pointer**, not a copy. Passing a temporary `std::string`'s `c_str()` will
leave a dangling pointer. Use string literals, exactly as `FlagRegistry` and `Routine::name` do.

### Publishing from your own code

One line, from wherever you already have the value:

```cpp
telemetry::TelemetryBus::instance().record("lift.heightInches", height);
```

No registration, no plumbing, no consumer changes. It appears in the SD log and the serial
stream automatically.

---

## Channel reference

Everything currently published:

### Drivetrain — from `DrivetrainVelocityController`, 100 Hz

| Channel | Type | Meaning |
|---|---|---|
| `drivetrain.left.targetRpm` | number | Commanded velocity, left |
| `drivetrain.left.actualRpm` | number | Measured velocity, left |
| `drivetrain.left.health` | text | `OK` / `STALLED` / `OVER_TEMP` / `DISCONNECTED` |
| `drivetrain.left.connectedMotors` | integer | Motors responding, left |
| `drivetrain.right.targetRpm` | number | Commanded velocity, right |
| `drivetrain.right.actualRpm` | number | Measured velocity, right |
| `drivetrain.right.health` | text | Right side health |
| `drivetrain.right.connectedMotors` | integer | Motors responding, right |
| `drivetrain.batteryLow` | boolean | Pack below the low-battery threshold |

### Odometry — from `OdometryFusion`, 200 Hz

| Channel | Type | Meaning |
|---|---|---|
| `odom.pose.x` / `.y` / `.heading` | number | Field pose |
| `odom.velocity.x` / `.y` / `.headingRate` | number | Field velocity |
| `odom.confidence` | text | `fullPod` / `partial` / `imeOnly` |
| `odom.forwardPods.healthyCount` | integer | Healthy forward pods |
| `odom.strafePods.healthyCount` | integer | Healthy strafe pods |
| `hal.leftIme.healthy` / `hal.rightIme.healthy` | boolean | Drive encoder health |
| `hal.imu.healthy` | boolean | At least one IMU ready |

### Subsystems — automatic

| Channel | Type | Meaning |
|---|---|---|
| *(the subsystem's own name)* | integer | Its current state ordinal |
| *(each registered flag name)* | boolean | Forwarded automatically by `FlagRegistry::setFlag()` |

Both come free from the framework — no per-subsystem wiring. So `exampleArm` and
`exampleArm.isExtended` appear in every log with no code beyond the subsystem itself.

### Autonomous

| Channel | Type | Meaning |
|---|---|---|
| `auton.routineName` | text | The routine that ran, or `"none"` |

---

## `SdLogger`

Writes a CSV file to the SD card on a dedicated **low-priority** background task.

```cpp
telemetry::SdLogger logger(telemetry::TelemetryBus::instance());
logger.start();
```

### How it works

1. Polls the bus every **40 ms** (~25 Hz) — that's the log's temporal resolution.
2. Formats each snapshot as one CSV row into a **RAM buffer** (pre-reserved 8 KB, so appending
   never reallocates).
3. Every **10 rows** (~400 ms), does a single buffered `fwrite` + `fflush` to the card.

Decoupling "how often we sample" from "how often we touch the card" is what bounds SD wear and
blocking-call frequency. **No timing-sensitive loop ever writes to the card** — they only call
`record()`.

### File naming

Files land in `/usd` as `lslog000.csv` … `lslog999.csv` — 8.3-safe, since FatFS on the V5 may
not support long filenames. Each run picks **the first index that doesn't already exist**, so
re-running never overwrites a previous match's log, even across power cycles.

### When a new file opens

On the **disabled → enabled** competition-state transition, **not** at boot. That means one file
per match rather than one per power-on.

The flag starts as "previously disabled" so a boot that begins already-enabled (untethered bench
testing) still opens a file on its first cycle.

### Format

```csv
timeMs,odom.pose.x,odom.pose.y,odom.pose.heading,drivetrain.left.targetRpm,...
1240,12.5000,36.2000,90.1000,150.0000,...
1280,12.5000,36.4000,90.1000,150.0000,...
```

First column is `pros::millis()`. Numbers are formatted to 4 decimal places.

Channels register in call order, so a channel that first appears a few cycles into a run would
arrive after the header was written. The logger **clamps each row to the header's column count**,
so the CSV stays rectangular and parseable.

### Failure handling

- **No SD card** — the file handle stays null, logging is skipped, a warning prints, and
  everything else continues normally.
- **Card pulled mid-match** — the `fwrite` return value is checked; logging stops cleanly rather
  than silently retrying a dead file every cycle for the rest of the run.

---

## `Dashboard`

A live brain-screen readout during driver control.

```cpp
telemetry::Dashboard dashboard(odometry, exampleArm);
dashboard.start();
```

Redraws at **8 Hz** (125 ms) and shows:

| Line | Content |
|---|---|
| 0 | Title |
| 1 | Battery voltage |
| 2 | Current pose (x, y, heading) |
| 3 | Odometry confidence tier |
| 4 | Subsystem state |
| box | Active-fault indicator |

### It gates itself

The dashboard only draws when `!is_disabled() && !is_autonomous()` — i.e. **only during driver
control**. The selector GUI owns the screen before the match and nothing draws during
autonomous, so the two never contend for `pros::screen`'s mutex.

### It takes direct references, not bus channels

`Dashboard` reads `OdometryFusion` and `ExampleArm` directly rather than pulling from the
telemetry bus. That's deliberate: the dashboard is a **curated, hand-picked** view of a few
things a driver needs mid-match, not a generic dump of every channel. The SD log is where
everything goes.

Consequence: adding a new mechanism to the dashboard means editing `dashboard.cpp` and
`dashboard_layout.hpp`. The fault indicator is currently keyed to `ExampleArmState::faulted`
specifically, so it needs revisiting when real subsystems replace the demo.

Layout constants (line indices, the fault box's pixel rect) live in
[`dashboard_layout.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/telemetry/dashboard_layout.hpp).

---

## `SerialLink`

Streams the bus over USB stdout as CSV — a header line, then one line per sample at **10 Hz** —
so a laptop-side script can tail the terminal and live-plot values.

```cpp
telemetry::SerialLink link(telemetry::TelemetryBus::instance());
link.start();
```

**Bench tuning only.** It is never started during normal competition operation — its only entry
point is [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/) (hold **Y** at boot).

This is the fastest way to see a step response while tuning the drivetrain. Pipe
`pros terminal` into a plotting script and watch `drivetrain.left.targetRpm` against
`drivetrain.left.actualRpm` in real time. See [Tuning Guide]({{ site.baseurl }}/guides/tuning/).

---

## Configuration

[`include/lightspeed/telemetry/telemetry_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/telemetry/telemetry_constants.hpp)

```cpp
// SD logger
kSdLoggerSampleIntervalMs   = 40;      // ~25 Hz — the log's temporal resolution
kSdLoggerRowsPerFlush       = 10;      // flush roughly every 400 ms
kSdLoggerBufferReserveBytes = 8192;    // pre-reserved so appends never reallocate
kSdLoggerDirectory          = "/usd";
kSdLoggerFilenameFormat     = "%s/lslog%03u.csv";
kSdLoggerMaxFileIndex       = 1000;

// Dashboard
kDashboardLoopPeriodMs = 125;          // ~8 Hz

// Serial link
kSerialLinkLoopPeriodMs = 100;         // ~10 Hz
```

Raise `kSdLoggerSampleIntervalMs` for longer matches or lower it for finer resolution while
debugging a fast transient. If you lower it a lot, consider raising `kSdLoggerRowsPerFlush` to
keep card writes at the same frequency.

---

**Next:** [Vision Layer]({{ site.baseurl }}/layers/vision/) — AprilTag pose correction.
