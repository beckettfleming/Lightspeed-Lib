---
title: Configuration Reference
parent: Reference
nav_order: 1
permalink: /reference/configuration/
---

Every tunable value in Lightspeed lives in a dedicated `*_constants.hpp` (or `config.hpp`) file.
No class hardcodes a magic number. This page is the map.

---

## The files

| File | Controls | Page |
|---|---|---|
| [`hal/config.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/hal/config.hpp) | **Every port number**, motor groups, gearsets | [HAL Layer]({{ site.baseurl }}/layers/hal/) |
| [`control/drivetrain_velocity_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/control/drivetrain_velocity_constants.hpp) | Drivetrain PIDF, slew, battery thresholds | [Control Layer]({{ site.baseurl }}/layers/control/) |
| [`odom/odometry_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/odom/odometry_constants.hpp) | Pod topology, wheel diameter, gear ratio | [Odometry Layer]({{ site.baseurl }}/layers/odometry/) |
| [`driver/driver_control_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/driver/driver_control_constants.hpp) | Drive mode, input curve, max RPM, accel rules | [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/) |
| [`motion/motion_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/motion/motion_constants.hpp) | Track width + every motion primitive's gains | [Motion Layer]({{ site.baseurl }}/layers/motion/) |
| [`auton/auton_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/auton_constants.hpp) | Field dimensions, field-to-screen mapping | [Autonomous Layer]({{ site.baseurl }}/layers/autonomous/) |
| [`auton/start_location.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/start_location.hpp) | Named starting positions | [Autonomous Layer]({{ site.baseurl }}/layers/autonomous/) |
| [`auton/screen_layout.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/screen_layout.hpp) | Selector GUI pixel layout | [Autonomous Layer]({{ site.baseurl }}/layers/autonomous/) |
| [`telemetry/telemetry_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/telemetry/telemetry_constants.hpp) | Logging rates, buffer sizes, filenames | [Telemetry Layer]({{ site.baseurl }}/layers/telemetry/) |
| [`telemetry/dashboard_layout.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/telemetry/dashboard_layout.hpp) | Dashboard line positions | [Telemetry Layer]({{ site.baseurl }}/layers/telemetry/) |
| [`vision/vision_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/vision/vision_constants.hpp) | Camera calibration, mount, tag map, gating | [Vision Layer]({{ site.baseurl }}/layers/vision/) |

---

## ⚠️ Values that must change together

This is the part that bites people. Several physical facts about the robot are expressed in more
than one file, and they must stay consistent.

### 1. The drivetrain top speed (currently 343 RPM)

```
   Real gearing: blue 600 RPM cartridge, geared down to ~343 RPM at the wheel
                                |
        +-----------------------+-----------------------+------------------+
        v                       v                       v                  v
  odom::kDriveImeConfig   motion::k...Kinematics   driver::kMaxDriveRpm   control::...pidf.kV
      .gearRatio               .gearRatio             = 343.0              = 35.0
    = 343.0/600.0            = 343.0/600.0                              (~12000mV / 343rpm)
```

| Where | Value | Meaning |
|---|---|---|
| `odom::kDriveImeConfig.gearRatio` | `343.0/600.0` | Odometry's distance conversion |
| `motion::kDrivetrainKinematics.gearRatio` | `343.0/600.0` | Motion's speed conversion |
| `driver::kMaxDriveRpm` | `343.0` | Full-stick target |
| `control::kDrivetrainVelocityConfig.pidf.kV` | `35.0` | ≈ 12000 mV / 343 RPM |

**Change your gearing and all four must change.** Getting `gearRatio` wrong in only one place
gives you odometry that disagrees with motion — the robot thinks it drove 20 inches when it drove
24.

### 2. Wheel diameter (currently 4.0 in)

- `odom::kDriveImeConfig.wheelDiameterInches`
- `motion::kDrivetrainKinematics.wheelDiameterInches`

Both must be the **drive wheel** diameter. (If you add tracking pods, their diameter is a
*separate* number that goes into each `PodConfig::ticksToInches` — don't conflate them.)

### 3. Screen resolution (480 × 272)

Declared twice on purpose — `auton::kScreenWidth/kScreenHeight` and
`telemetry::kDashboardScreenWidth/Height`. This duplication is deliberate (each module owns its
own hardware constants) and won't change unless VEX ships a different brain.

---

## Quick reference: what to change for a common task

| Task | Edit |
|---|---|
| Rewire a motor | `hal/config.hpp` — the `port` namespace |
| Add a motor to a side | `hal/config.hpp` — that side's `MotorGroupConfig.ports` |
| Reverse a motor | Negate its port number in `hal/config.hpp` |
| Change drive cartridges | `hal/config.hpp` gearset **+ all four values in §1 above** |
| Drivetrain feels sluggish/oscillating | `control/drivetrain_velocity_constants.hpp` |
| Robot drives too far / not far enough | `odom/odometry_constants.hpp` — wheel diameter and gear ratio |
| Turns overshoot | `motion/motion_constants.hpp` — `kTurnToHeadingConfig.pidf` |
| Turns are the wrong *amount* | `motion/motion_constants.hpp` — `trackWidthInches` |
| Driver wants a different drive scheme | `driver/driver_control_constants.hpp` — `kDriveMode` |
| Driver wants finer low-speed control | `driver/driver_control_constants.hpp` — `curveExponent` |
| Robot tips when a mechanism is up | `driver/driver_control_constants.hpp` — add an `AccelLimitRule` |
| Add a start position | `auton/start_location.hpp` |
| Different field size | `auton/auton_constants.hpp` |
| Log more/less often | `telemetry/telemetry_constants.hpp` |
| Camera moved | `vision/vision_constants.hpp` — `kAiVisionMountOffset` |

---

## Annotated: `hal/config.hpp`

```cpp
namespace port {
inline constexpr std::int8_t  kLeftDriveFront  =   1;
inline constexpr std::int8_t  kLeftDriveRear   =   9;
inline constexpr std::int8_t  kRightDriveFront =  -2;   // negative = reversed
inline constexpr std::int8_t  kRightDriveRear  = -10;   // negative = reversed
inline constexpr std::int8_t  kIntakeFront     =   4;
inline constexpr std::int8_t  kIntakeRear      =   5;
inline constexpr std::uint8_t kPrimaryImu      =   3;
inline constexpr std::uint8_t kSecondaryImu    =   8;
inline constexpr std::int8_t  kExampleArmMotor =  12;   // demo placeholder
inline constexpr std::uint8_t kAiVisionSensor  =  13;
}
```

Ports are 1–21. **Negative reverses the motor internally.** IMU and AI Vision ports are
unsigned — no reversal concept applies.

Motor groups pair ports with a gearset and encoder units:

| Group | Ports | Gearset |
|---|---|---|
| `kLeftDriveGroup` | 1, 9 | blue (600 RPM) |
| `kRightDriveGroup` | -2, -10 | blue (600 RPM) |
| `kIntakeFrontGroup` | 4 | green (200 RPM) |
| `kIntakeRearGroup` | 5 | green (200 RPM) |
| `kExampleArmGroup` | 12 | green (200 RPM) |

Also here: `kAiVisionSensor.tagFamily` (currently `tag_16H5`) — confirm against the season's
actual field elements.

---

## Annotated: `control/drivetrain_velocity_constants.hpp`

| Field | Default | What it does | Tune? |
|---|---|---|---|
| `pidf.kP` | 20.0 | mV per RPM of error | **Yes** |
| `pidf.kI` | 0.0 | Integral | Only after kP/kD |
| `pidf.kD` | 0.0 | Damping | **Yes** |
| `pidf.kV` | 35.0 | mV per target RPM — the main term | **Yes, first** |
| `pidf.kA` | 0.0 | Acceleration feedforward | Optional |
| `pidf.kS` | 300.0 | mV to break static friction | **Yes** |
| `pidf.integralZone` | 0.0 | Zone gating (disabled) | With kI |
| `pidf.integralMax` | 4000.0 | Windup clamp, mV | Rarely |
| `pidf.settleTolerance` | 10.0 | RPM | Rarely |
| `pidf.settleCycles` | 10 | ~100 ms at 100 Hz | Rarely |
| `maxVoltageSlewRatePerSecond` | 240000.0 | Full 12 V swing in ~50 ms | **Yes** |
| `nominalBatteryMillivolts` | 12000.0 | Reference the gains were tuned at | No |
| `lowBatteryMillivolts` | 11000.0 | Generic V5 "getting low" threshold | Rarely |
| `loopPeriodMs` | 10 | 100 Hz | **Don't raise** — matches the motor's own rate |

---

## Annotated: `motion/motion_constants.hpp`

### Shared geometry

| Field | Default | Notes |
|---|---|---|
| `trackWidthInches` | 12.0 | **Measure it** — scales every turn |
| `wheelDiameterInches` | 4.0 | Must match odometry |
| `gearRatio` | 343/600 | Must match odometry |

### `kTurnToHeadingConfig`

| Field | Default | Notes |
|---|---|---|
| `pidf.kP` | 0.6 | deg/s command per degree of error |
| `pidf.kD` | 0.05 | Damping |
| `pidf.settleTolerance` | 2.0 | degrees |
| `pidf.settleCycles` | 8 | ~160 ms at 50 Hz |
| `timeoutSeconds` | 3.0 | |
| `loopPeriodMs` | 20 | 50 Hz |

`kV`/`kA`/`kS` are 0 — no feedforward for a static heading target.

### `kDriveStraightDistanceConfig`

| Field | Default | Notes |
|---|---|---|
| `distancePidf.kP` | 3.0 | (in/s) per inch of error |
| `distancePidf.kD` | 0.2 | |
| `distancePidf.kV` | **1.0** | Passes the profile's velocity straight through — keep at ~1.0 |
| `distancePidf.settleTolerance` | 0.5 | inches |
| `motionProfile.maxVelocity` | 40.0 | in/s |
| `motionProfile.maxAcceleration` | 80.0 | in/s² |
| `motionProfile.maxJerk` | 400.0 | in/s³ — S-curve on long enough moves |
| `headingCorrectionKP` | 2.0 | deg/s of trim per degree of drift |
| `timeoutSeconds` | 5.0 | |

### `kMoveToPoseConfig`

| Field | Default | Notes |
|---|---|---|
| `leadFraction` | 0.4 | Higher = earlier, more aggressive turn-in |
| `positionToleranceInches` | 2.0 | |
| `headingToleranceDegrees` | 3.0 | |
| `settleCycles` | 8 | Both tolerances must hold |
| `timeoutSeconds` | 6.0 | |

### `kPurePursuitConfig`

| Field | Default | Notes |
|---|---|---|
| `minLookaheadInches` | 6.0 | Too small → weaving |
| `maxLookaheadInches` | 18.0 | Too large → corner cutting |
| `lookaheadSpeedGain` | 0.3 | Extra inches of lookahead per in/s |
| `positionToleranceInches` | 2.0 | |
| `timeoutSeconds` | 8.0 | |
| `smoothing.injectionSpacingInches` | 2.0 | Point density |
| `smoothing.weightData` | 0.5 | Pull toward the original route |
| `smoothing.weightSmooth` | 0.25 | Pull toward neighbors — **keep data+smooth < 1** |
| `smoothing.smoothingToleranceInches` | 0.001 | Convergence threshold |
| `smoothing.maxSmoothingIterations` | 100 | Safety cap |

---

## Annotated: `driver/driver_control_constants.hpp`

| Field | Default | Notes |
|---|---|---|
| `kDriveMode` | `arcade` | `tank` / `arcade` / `curvature` — driver preference |
| `kInputProfileConfig.curveExponent` | 1.3 | Higher = softer low-speed response |
| `kInputProfileConfig.deadband` | 0.05 | Ignore stick drift |
| `kMaxDriveRpm` | 343.0 | **Must match the drivetrain `kV` assumption** |
| `kDriveAccelLimitConfig.defaultMaxRpmPerSecond` | 12000.0 | Effectively unlimited |
| `kDriveAccelLimitConfig.rules` | one demo rule | Flag name → max RPM/s |

---

## Placeholder audit

Every value below is a **clearly-marked placeholder** that needs a real measurement or
season-specific information. See **[Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/)** for the working list with checkboxes.

| Category | Location |
|---|---|
| All port numbers | `hal/config.hpp` |
| Drive gearset + gear ratio | `hal/config.hpp`, `odom/`, `motion/` |
| Every drivetrain gain | `control/drivetrain_velocity_constants.hpp` |
| Track width | `motion/motion_constants.hpp` |
| Every motion gain | `motion/motion_constants.hpp` |
| Max RPM, accel limits, input curve | `driver/driver_control_constants.hpp` |
| Start locations | `auton/start_location.hpp` |
| Field dimensions | `auton/auton_constants.hpp` |
| Demo routines | `auton/demo_routines.*` |
| Demo subsystem | `subsystem/demo/example_arm.*` |
| All camera calibration + tag map | `vision/vision_constants.hpp` |
