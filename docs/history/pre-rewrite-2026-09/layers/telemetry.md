# `lightspeed::telemetry` — telemetry and logging

One passive named-channel bus that every layer writes to on its own update
cycle, plus three independent consumers that poll it at their own rates: an SD
card CSV logger, a brain-screen dashboard, and a serial link for laptop-side
plotting.

The split matters. Producers never block on I/O — `record()` is a plain in-memory
write. Consumers never dictate sample rates to producers. Adding a new consumer
requires touching no producer.

## `TelemetryBus`

```cpp
static TelemetryBus& instance();

void record(const char* name, double value);
void record(const char* name, bool value);
void record(const char* name, std::int32_t value);
void record(const char* name, const char* value);

static constexpr std::uint8_t kMaxChannels = 32;

struct Snapshot {
    std::array<ChannelSnapshot, kMaxChannels> channels{};
    std::uint8_t count = 0;
};

Snapshot getSnapshot() const;
```

```cpp
enum class ChannelType : std::uint8_t { number, integer, boolean, text };

struct ChannelSnapshot {
    const char* name;
    ChannelType type;
    double      number;
    std::int32_t integer;
    bool        boolean;
    const char* text;
};
```

A channel is **registered on first use**, in call order, and that order is stable
thereafter — which is what lets the CSV logger write a header once and keep
columns aligned.

`record()` is safe from any task, every cycle: an array scan under a short-lived
mutex, no allocation. `getSnapshot()` is a fixed-size array copy, equally safe.

**Channel names must be stable-lifetime strings** — string literals. The bus
stores the pointer, not a copy. Same rule as `FlagRegistry` and `Routine::name`.

> ⚠️ **32 channels is a hard cap.** The library already publishes ~24 (see the
> table below, plus one per subsystem and one per flag). Adding a subsystem adds
> at least two. Count before you add a batch of new channels; the 33rd is
> silently dropped.

### Channels published by the library

| Channel | Type | Producer |
| --- | --- | --- |
| `drivetrain.left.targetRpm` / `.right.targetRpm` | number | `control` |
| `drivetrain.left.actualRpm` / `.right.actualRpm` | number | `control` |
| `drivetrain.left.health` / `.right.health` | text | `control` |
| `drivetrain.left.connectedMotors` / `.right.connectedMotors` | integer | `control` |
| `drivetrain.batteryLow` | boolean | `control` |
| `odom.pose.x` / `.y` / `.heading` | number | `odom` |
| `odom.velocity.x` / `.y` / `.headingRate` | number | `odom` |
| `odom.confidence` | text | `odom` |
| `odom.forwardPods.healthyCount` / `odom.strafePods.healthyCount` | integer | `odom` |
| `hal.leftIme.healthy` / `hal.rightIme.healthy` / `hal.imu.healthy` | boolean | `odom` |
| `auton.routineName` | text | `main.cpp` |
| *(subsystem name)* | integer | `Subsystem::update()` — raw state ordinal |
| *(flag name)* | boolean | `FlagRegistry::setFlag()` |

The last two rows are why subsystems and flags need no per-instance telemetry
wiring: every subsystem publishes its state under its own `SubsystemConfig::name`
automatically, and every flag publishes under its own name on every set.

## `SdLogger`

A dedicated low-priority task polls the bus at a fixed rate, formats each
snapshot as one CSV row into a RAM buffer, and flushes several rows at a time to
the SD card.

```cpp
explicit SdLogger(const TelemetryBus& bus);
void start();   // no-op if already started
```

Non-copyable, non-movable.

* **Sample interval** 40 ms (~25 Hz) — the log's temporal resolution.
* **Flush** every 10 rows (~400 ms). Decoupling sampling from flushing is what
  bounds SD wear and blocking-call frequency.
* **Buffer** 8 KB reserved once at construction, so appending never reallocates.
* **Files** `/usd/lslog000.csv` … `/usd/lslog999.csv`, 8.3-safe for the V5's
  FatFS.

**A new file opens on the disabled → enabled competition transition**, not at
boot — so re-running a match never overwrites the previous one, and a run's file
is the first index that does not already exist, even across power cycles. The
edge-detect flag starts as "previously disabled," so a boot that begins already
enabled (untethered bench testing) still opens a file on its first cycle.

Degrades cleanly with **no SD card present** — the file handle stays null and
logging is skipped. It also checks `fwrite`'s return value, so a card pulled
mid-match stops logging cleanly instead of silently retrying a dead handle for
the rest of the run.

## `Dashboard`

A live brain-screen readout during driver control.

```cpp
Dashboard(const odom::OdometryFusion& odometry,
          const subsystem::demo::ExampleArm& exampleArm);   // ⚠️ placeholder reference
void start();
```

Redraws at 125 ms (~8 Hz). Shows battery voltage, current pose, odometry
confidence tier, subsystem state, and a fault indicator box.

Unlike the SD logger, it takes **direct references** to what it displays rather
than reading the bus — a curated, hand-picked view rather than a generic dump.
That is the same pattern `AutonomousContext` and `SelectorGui` use.

**It gates itself on competition state so it never draws outside the
driver-control period.** The selector GUI owns the screen through
start-location/routine confirmation, and nothing draws during autonomous, so the
two never contend for `pros::screen`'s mutex.

> ⚠️ Its fault indicator is keyed to `ExampleArmState::faulted` specifically.
> Revisit when real subsystems replace the demo.

Layout constants (line indices, not pixel offsets, for the text rows) are in
`dashboard_layout.hpp`.

## `SerialLink`

Bench-tuning only. Streams the bus over the existing USB stdout as CSV — one
header line, then one data line per sample, at 100 ms (~10 Hz) — so a
laptop-side script can tail `pros terminal` and live-plot values.

```cpp
explicit SerialLink(const TelemetryBus& bus);
void start();
```

**Not part of competition operation.** It is constructed in `initialize()` but
only ever started inside
[diagnostic mode](diagnostics.md) — which is what gave it a real entry point
instead of leaving it permanently unreachable.

## Constants — `telemetry_constants.hpp`

```cpp
inline constexpr std::uint32_t kSdLoggerSampleIntervalMs = 40;    // ~25 Hz
inline constexpr std::uint16_t kSdLoggerRowsPerFlush     = 10;    // ~400 ms
inline constexpr std::size_t   kSdLoggerBufferReserveBytes = 8192;
inline constexpr const char*   kSdLoggerDirectory        = "/usd";
inline constexpr const char*   kSdLoggerFilenameFormat   = "%s/lslog%03u.csv";
inline constexpr unsigned int  kSdLoggerMaxFileIndex     = 1000;

inline constexpr std::uint32_t kDashboardLoopPeriodMs    = 125;   // ~8 Hz
inline constexpr std::uint32_t kSerialLinkLoopPeriodMs   = 100;   // ~10 Hz
```

These are deliberate engineering choices rather than placeholders — but if you
need finer log resolution for a tuning session, lowering
`kSdLoggerSampleIntervalMs` is the knob.
