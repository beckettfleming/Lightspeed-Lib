---
title: HAL
parent: Layers
nav_order: 1
permalink: /layers/hal/
---

**Namespace:** `lightspeed::hal` · **Headers:** `include/lightspeed/hal/`

The Hardware Abstraction Layer is the only part of Lightspeed that talks to PROS device APIs.
Everything above it works with `hal::MotorGroup`, `hal::Imu`, `hal::RotationSensor`, and
`hal::AiVisionSensor` instead of `pros::Motor` and friends.

**Design rule:** the HAL stays *unit-agnostic*. It reports raw motor degrees and raw sensor
centidegrees, never inches. Converting to physical units (which needs wheel diameter and gear
ratio) is the odometry layer's job. This keeps the HAL reusable for any mechanism.

---

## The port map — `config.hpp`

> **This file is the only place a raw port number may appear anywhere in the codebase.**
> Rewiring the robot should mean editing this one file and nothing else.

[`include/lightspeed/hal/config.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/hal/config.hpp)

### Current port assignments

| Logical device | Port(s) | Notes |
|---|---|---|
| Left drive front | `1` | |
| Left drive rear | `9` | |
| Right drive front | `-2` | negative = reversed |
| Right drive rear | `-10` | negative = reversed |
| Intake front | `4` | |
| Intake rear | `5` | |
| Primary IMU | `3` | |
| Secondary IMU | `8` | |
| Example arm (demo) | `12` | placeholder subsystem — see [Subsystem Layer]({{ site.baseurl }}/layers/subsystem/) |
| AI Vision Sensor | `13` | |

> ⚠️ **Every one of these is an unconfirmed placeholder.** Confirm them against your actual
> wiring before running anything. See [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/).

### A negative port means "reversed"

V5 smart ports are numbered 1–21. A **negative** port number tells `pros::MotorGroup` to invert
that motor internally. That is why the right side uses `-2` and `-10`: with the motors mounted
mirror-image, a negative port lets both sides be commanded with the *same sign* of voltage and
still turn the robot forward.

### Motor group definitions

A `MotorGroupConfig` bundles a name, its ports, its gearset, and its encoder units:

```cpp
struct MotorGroupConfig {
    const char* name;
    std::vector<std::int8_t> ports;
    pros::v5::MotorGears gearset;
    pros::v5::MotorUnits encoderUnits;
};
```

Currently defined:

| Config | Ports | Gearset | Purpose |
|---|---|---|---|
| `kLeftDriveGroup` | 1, 9 | blue (600 RPM) | Left drivetrain side |
| `kRightDriveGroup` | -2, -10 | blue (600 RPM) | Right drivetrain side |
| `kIntakeFrontGroup` | 4 | green (200 RPM) | Front intake |
| `kIntakeRearGroup` | 5 | green (200 RPM) | Rear intake |
| `kExampleArmGroup` | 12 | green (200 RPM) | Demo subsystem (placeholder) |

The drive gearset assumes **blue 6:1 (600 RPM) cartridges with an external reduction to roughly
343 RPM at the wheel.** That 343 number recurs in three other files — see
[Configuration Reference]({{ site.baseurl }}/reference/configuration/) for why they must all be changed together.

### Adding a new device

1. Add its port constant to the `port` namespace.
2. Add a `MotorGroupConfig` (or `ImuConfig` / `RotationSensorConfig` / `AiVisionConfig`) below it.
3. Construct the HAL wrapper from that config in `initialize()` in `src/main.cpp`.

That's it — no other file needs to learn about the port.

---

## `MotorGroup`

Wraps a ganged set of motors driven by raw voltage. Used for the drivetrain sides, the intakes,
and any subsystem mechanism.

```cpp
hal::MotorGroup leftDrive(hal::config::kLeftDriveGroup);

double pos   = leftDrive.getPositionRaw();    // averaged encoder position, raw units
double rpm   = leftDrive.getVelocityRpm();    // averaged measured velocity
leftDrive.writeVoltage(6000);                  // millivolts, clamped to +/-12000
```

### Reading state

| Method | Returns |
|---|---|
| `getPositionRaw()` | Encoder position averaged across the group, in the group's configured units |
| `getVelocityRpm()` | Measured velocity averaged across the group, in RPM |
| `getHealth()` | One `HealthStatus` value for the whole group |
| `getMotorCount()` | How many motors are configured (fixed at construction) |
| `getConnectedMotorCount()` | How many are actually responding right now |
| `name()` | The group's name string, for logging and telemetry |

Averaging skips non-finite readings, so one disconnected motor reporting `PROS_ERR_F` doesn't
poison the whole group's number.

### Health detection

```cpp
enum class HealthStatus : std::uint8_t { ok, stalled, overTemperature, disconnected };
```

`getHealth()` checks in this order:

1. **Disconnected** — any motor reports a non-finite temperature. Checked first, because a
   disconnected motor would otherwise also look "not over temp" and "not stalled".
2. **Over temperature** — any motor reports over-temp.
3. **Stalled** — any motor is being commanded ≥ 25% of full voltage (3000 mV) but is turning
   slower than 3 RPM. That is the signature of a jam or severe overload.
4. Otherwise **ok**.

### Partial vs. total disconnection

`getHealth()` is deliberately coarse: it returns `disconnected` the instant *one* motor drops
out, collapsing "1 of 2 gone" and "2 of 2 gone" into one status.

That's not enough information for a fail-safe, so pair it with the counts:

```cpp
if (group.getHealth() == HealthStatus::disconnected &&
    group.getConnectedMotorCount() == 0) {
    // Truly dead — stop this side.
} else {
    // Degraded but still drivable — keep going on the motors that remain.
}
```

This is exactly what `DrivetrainVelocityController` and `Subsystem::driveToPosition()` do. A
robot with one dead drive motor per side should still be able to limp off the field, not stop.

### Voltage clamping

`writeVoltage()` clamps to ±12000 mV before it reaches the motors. Every voltage path in the
project goes through this method, so no caller can command an out-of-range value even by
mistake.

---

## `Imu`

Wraps a single V5 Inertial Sensor. Dual-IMU averaging is *not* here — that lives one layer up
in `odom::IMUSource`, because it's a fusion concern rather than a device concern.

```cpp
hal::Imu imu(hal::config::kPrimaryImu.port);

imu.calibrate(true);                        // blocking, ~2-3 seconds
bool ready = imu.isReady();
double heading = imu.getHeadingDegrees();            // [0, 360), clockwise-positive
double total   = imu.getContinuousHeadingDegrees();  // unbounded, no wraparound
```

### The two heading methods, and which to use

| Method | Range | Use for |
|---|---|---|
| `getHeadingDegrees()` | `[0, 360)` | **Display only** |
| `getContinuousHeadingDegrees()` | unbounded | **Any delta math** |

Use the continuous one whenever you subtract two readings. If you use the wrapped one, a robot
turning through 0° produces a 359° "delta" instead of a 1° one, and your odometry jumps across
the field.

`OdometryFusion` follows this rule strictly — it integrates a continuous heading internally and
only wraps to `[0, 360)` when publishing a pose.

### Calibration matters

Both heading methods return `0.0` if the sensor isn't ready. A match must never begin trusting
a mid-calibration heading, which is why `initialize()` calls `calibrate(true)` (blocking) on
both IMUs before anything else runs. Keep the robot still for those 2–3 seconds.

---

## `RotationSensor`

Wraps a single V5 Rotation Sensor, intended for odometry tracking-wheel pods.

```cpp
hal::RotationSensor pod(portNumber);   // negative port reverses direction

double raw = pod.getPositionCentidegrees();  // 0.01-degree units, unconverted
bool ok    = pod.isHealthy();
```

Returns `0.0` when the sensor isn't reporting valid data. Inch conversion happens in
`odom::TrackingWheelSource` via `PodConfig::ticksToInches`.

> **Note:** the current robot has **no tracking-wheel pods** — odometry runs on drive encoders
> plus dual IMU. This wrapper and the whole pod-handling path in `OdometryFusion` are fully
> built and ready if pods are added later. See [Odometry Layer]({{ site.baseurl }}/layers/odometry/).

---

## `AiVisionSensor`

Wraps the VEX AI Vision Sensor, filtered to **AprilTag detections only**. Color, code, and
AI-object detection modes are never enabled — the vision layer only needs tags.

```cpp
hal::AiVisionSensor camera(hal::config::kAiVisionSensor);

hal::TagDetectionList tags = camera.getDetectedTags();
for (std::uint8_t i = 0; i < tags.count; ++i) {
    const hal::TagDetection& tag = tags.tags[i];
    // tag.id, and four corners: (x0,y0) .. (x3,y3), in pixels
}
```

Consistent with the rest of the HAL, this reports **raw pixel corners** and nothing else.
Turning corners into a bearing, distance, and skew is `vision::solveRelativePose()`'s job.

- `kMaxTagDetections` is 24 — the hardware's own cap on simultaneous detections.
- `TagDetectionList` is a fixed-capacity array: no allocation on the hot path.
- The internal `pros::AIVision` member is `mutable` because PROS's read methods aren't
  `const`-qualified, even though they're plain reads.

The tag family is set in `config.hpp` (`kAiVisionSensor.tagFamily`, currently `tag_16H5`) and
needs confirming against the season's actual field elements.

---

## What the HAL deliberately does *not* do

- **No unit conversion.** Raw degrees and centidegrees out; inches are a higher layer's problem.
- **No caching or polling loop.** Every read is a cheap, side-effect-free query, safe to call
  from multiple tasks at different rates. A shared caching layer could be added in front later
  without changing this interface.
- **No control logic.** No PID, no ramping, no limits beyond the ±12000 mV clamp.

---

**Next:** [Control Layer]({{ site.baseurl }}/layers/control/) — what turns HAL reads into HAL writes.
