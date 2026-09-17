# `lightspeed::hal` — hardware abstraction

Thin, unit-agnostic wrappers around the PROS device APIs. Everything above this
layer talks to hardware through these classes and never touches `pros::Motor`,
`pros::Imu`, `pros::Rotation`, or `pros::AIVision` directly.

Two rules define this layer:

1. **`config.hpp` is the only file in the project with a raw port number in it.**
   Rewiring the robot is a one-file edit.
2. **HAL does not convert units.** It reports raw motor degrees and raw
   centidegrees. Wheel diameter and gear ratio are the odometry layer's problem;
   inches-per-second to RPM is the motion layer's. The same `MotorGroup` backs a
   drivetrain and an arm precisely because it knows it is neither.

## `config.hpp` — the port map

```cpp
namespace lightspeed::hal::config::port {
    inline constexpr std::int8_t kLeftDriveFront   =  1;
    inline constexpr std::int8_t kRightDriveFront  = -4;  // negative = reversed
    // ...
}
```

A **negative port number** tells PROS to reverse that device internally, so
every motor in a group can be commanded with the same sign and still turn the
output shaft the same physical way.

Four config struct types describe logical devices:

| Struct | Fields | Used by |
| --- | --- | --- |
| `MotorGroupConfig` | `name`, `ports`, `gearset`, `encoderUnits` | `MotorGroup` |
| `RotationSensorConfig` | `name`, `port` | paired with `RotationSensor` |
| `ImuConfig` | `name`, `port` | paired with `Imu` |
| `AiVisionConfig` | `name`, `port`, `tagFamily` | `AiVisionSensor` |

Add a new mechanism by adding ports to `port::` and a `MotorGroupConfig`
alongside `kLeftDriveGroup` / `kRightDriveGroup`.

> ⚠️ Every port number, the drive gearset, and the AprilTag family are
> placeholders. See [checklist § 1](../BUCKET_B_CHECKLIST.md).

## `MotorGroup`

A ganged group of V5 motors controlled by **raw voltage**.

```cpp
explicit MotorGroup(const config::MotorGroupConfig& config);

double       getPositionRaw()   const;  // averaged, in the group's configured units
double       getVelocityRpm()   const;  // averaged, measured
void         writeVoltage(std::int32_t millivolts) const;  // clamped to ±12000
HealthStatus getHealth()        const;
std::uint8_t getMotorCount()    const;  // fixed at construction
std::uint8_t getConnectedMotorCount() const;
const char*  name()             const;
```

All reads are cheap, side-effect-free, and safe to call from multiple tasks at
different rates.

### `HealthStatus`

```cpp
enum class HealthStatus : std::uint8_t { ok, stalled, overTemperature, disconnected };
const char* toString(HealthStatus status);
```

`getHealth()` is deliberately **coarse**: it reports `disconnected` the instant
*one* motor in the group drops out, collapsing "1 of 3 gone" and "3 of 3 gone"
into the same status.

To tell those apart — and you usually want to, since a partially-disconnected
side is still drivable — compare `getConnectedMotorCount()` against
`getMotorCount()`. That is exactly what the drivetrain controller and the
subsystem base class do:

```cpp
const bool mustStop = health == HealthStatus::stalled
                   || health == HealthStatus::overTemperature
                   || (health == HealthStatus::disconnected
                       && motors.getConnectedMotorCount() == 0);
```

A stall or overheat stops immediately. A disconnect stops only when *every*
motor is gone.

## `Imu`

Wraps **one** physical V5 Inertial Sensor. Dual-IMU averaging lives one layer up
in `odom::IMUSource`.

```cpp
explicit Imu(std::uint8_t port);

void   calibrate(bool blocking = false) const;  // ~2-3 s when blocking
bool   isReady()                        const;
double getHeadingDegrees()              const;  // [0, 360), clockwise-positive
double getContinuousHeadingDegrees()    const;  // unbounded, no wraparound
```

**Use `getContinuousHeadingDegrees()` whenever you compute a delta.** The
bounded version wraps at 360 and will produce a spurious ±360° jump. Both return
`0.0` before calibration finishes — a match must never start trusting a
mid-calibration heading, which is why `main.cpp` calibrates blocking during
`initialize()`.

## `RotationSensor`

```cpp
explicit RotationSensor(std::int8_t port);   // negative reverses direction

double getPositionCentidegrees() const;      // 0.01° units, unconverted
bool   isHealthy()               const;
```

Returns `0.0` when not reporting valid data.

## `AiVisionSensor`

Wraps the VEX AI Vision Sensor, **filtered to AprilTag detections only** — the
sensor's colour/code/AI-object modes are never enabled here, because
`lightspeed::vision` only needs tags.

```cpp
explicit AiVisionSensor(const config::AiVisionConfig& config);

TagDetectionList getDetectedTags() const;   // fixed capacity, no allocation
bool             isHealthy()       const;
```

```cpp
struct TagDetection {
    std::uint8_t id;
    double x0, y0, x1, y1, x2, y2, x3, y3;   // pixel corners, sensor's own order
};

inline constexpr std::uint8_t kMaxTagDetections = 24;   // hardware's own cap

struct TagDetectionList {
    std::array<TagDetection, kMaxTagDetections> tags{};
    std::uint8_t count = 0;
};
```

Consistent with the rest of HAL, this hands back **raw pixel corners**. Solving
those into a bearing, distance, and skew is
[`vision::solveRelativePose()`](vision.md)'s job.

## Telemetry published

| Channel | Type | Source |
| --- | --- | --- |
| `drivetrain.left.health` / `.right.health` | text | via `control` |
| `drivetrain.left.connectedMotors` / `.right.connectedMotors` | integer | via `control` |
| `hal.leftIme.healthy` / `hal.rightIme.healthy` / `hal.imu.healthy` | boolean | via `odom` |
