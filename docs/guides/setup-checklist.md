---
title: Setup Checklist
parent: Guides
nav_order: 1
permalink: /guides/setup-checklist/
---

Everything in this list is **implemented in code and working** — it's just populated with a
clearly-marked placeholder, because finishing it correctly needs a physical measurement, a
hardware confirmation, or season-specific information that doesn't exist yet.

Nothing here is "wrong". It builds, runs, and behaves correctly. It just isn't **true** yet.
Filling one in means editing the named constant at the named location — nothing more.

> This is the checklist form of [`docs/BUCKET_B_CHECKLIST.md`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/docs/BUCKET_B_CHECKLIST.md),
> updated to match the current code. Note that the config has evolved since that document was
> written — the robot no longer uses tracking-wheel pods, and intake motors have been added.

---

## 1 · Port wiring

**File:** [`include/lightspeed/hal/config.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/hal/config.hpp)
**Page:** [HAL Layer]({{ site.baseurl }}/layers/hal/)

Every raw port number in the codebase lives in this one file.

- [ ] **Drive motor ports** — `kLeftDriveFront` (1), `kLeftDriveRear` (9),
      `kRightDriveFront` (-2), `kRightDriveRear` (-10). Confirm once the drivetrain is wired.
      Remember: a **negative** port reverses that motor.
- [ ] **Intake ports** — `kIntakeFront` (4), `kIntakeRear` (5).
- [ ] **IMU ports** — `kPrimaryImu` (3), `kSecondaryImu` (8). These are currently **unwired
      placeholders**; confirm once the IMUs are mounted.
- [ ] **AI Vision Sensor port** — `kAiVisionSensor` (13). Confirm once mounted.
- [ ] **AprilTag family** — `kAiVisionSensor.tagFamily` (currently `tag_16H5`). Confirm against
      the season's actual field elements.
- [ ] **Drive gearset** — `kLeftDriveGroup` / `kRightDriveGroup` currently assume blue
      (6:1, 600 RPM) cartridges with an external reduction to ~343 RPM output.
- [ ] **`kExampleArmMotor`** (12) — reuse or remove once a real subsystem replaces the demo
      (see §6).

---

## 2 · Physical geometry

**Files:** [`odom/odometry_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/odom/odometry_constants.hpp),
[`motion/motion_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/motion/motion_constants.hpp)

- [ ] **`trackWidthInches`** (currently 12.0) — measure the distance between the left and right
      wheel contact patches. **This directly scales every turn**; a wrong value cannot be fixed
      by tuning gains.
- [ ] **`wheelDiameterInches`** (currently 4.0, the robot's drive omnis) — confirm. Appears in
      **both** `odom::kDriveImeConfig` and `motion::kDrivetrainKinematics`; they must match.
- [ ] **`gearRatio`** (currently 343/600) — confirm the actual external reduction. Also appears
      in both files, and implies `driver::kMaxDriveRpm` and the drivetrain's `kV`. See
      [Configuration Reference §1]({{ site.baseurl }}/reference/configuration/#1-the-drivetrain-top-speed-currently-343-rpm).

### If you add tracking wheels later

`kOdometryTopology.pods` is intentionally **empty** — this robot runs on drive encoders plus dual
IMU. The full pod machinery is built and ready. If pods get added:

- [ ] Add their Rotation Sensor ports to `hal/config.hpp`.
- [ ] Add a `PodConfig` per pod: name, role, measured `offsetInches`, and `ticksToInches`
      (= wheel circumference / 36000).
- [ ] Construct `hal::RotationSensor` + `TrackingWheelSource` per pod in `initialize()` and pass
      the pointers to `OdometryFusion` instead of the empty vector.
- [ ] **Verify each `offsetInches` sign** by rotating in place — if x/y drift during a pure
      rotation, a sign is flipped. See [Coordinate System]({{ site.baseurl }}/reference/coordinates/).

---

## 3 · Control gains

**Every gain in this project needs on-robot tuning.** None can be simulated correctly without
the real drivetrain's mass, friction, and battery behavior.

Follow **[Tuning Guide]({{ site.baseurl }}/guides/tuning/)** — the order matters.

### [`control/drivetrain_velocity_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/control/drivetrain_velocity_constants.hpp)

- [ ] `kV` (35.0) — **tune first**, it does most of the work
- [ ] `kS` (300.0) — static friction
- [ ] `kP` (20.0)
- [ ] `kD` (0.0)
- [ ] `kI` (0.0) — usually leave at zero
- [ ] `maxVoltageSlewRatePerSecond` (240000.0)

`lowBatteryMillivolts` (11000.0) is a reasonable generic V5-pack default — safe to leave unless
real match data says otherwise.

### [`motion/motion_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/motion/motion_constants.hpp)

- [ ] `kDrivetrainKinematics.trackWidthInches` — see §2
- [ ] Every gain in `kTurnToHeadingConfig`
- [ ] Every gain in `kDriveStraightDistanceConfig` (including the motion-profile limits)
- [ ] Every gain in `kPurePursuitConfig` (lookahead is the main knob)
- [ ] `kMoveToPoseConfig.leadFraction` (0.4)

### [`driver/driver_control_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/driver/driver_control_constants.hpp)

- [ ] `kMaxDriveRpm` (343.0) — tied to real gearing; confirm together with the drivetrain `kV`
- [ ] `kDriveAccelLimitConfig.defaultMaxRpmPerSecond` (12000.0)
- [ ] `kDriveMode` and `curveExponent` — **driver preference**, not a measurement

---

## 4 · AprilTag vision

**File:** [`vision/vision_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/vision/vision_constants.hpp)
**Page:** [Vision Layer]({{ site.baseurl }}/layers/vision/)

None of this is wired into the live competition path — precisely *because* it's all placeholder.
Everything below is validated through [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/).

- [ ] **`kAiVisionCalibration`** — real focal length and principal point (factory or checkerboard
      calibration), plus your actual printed `tagSizeInches`.
- [ ] **`kAiVisionMountOffset`** — measure once the camera is physically mounted. A wrong offset
      puts a constant error into every correction.
- [ ] **`kTagWorldMap`** — replace with the real season's AprilTag placements once published.
- [ ] **`kVisionGatingConfig.stableFrameThreshold`** (5) and **`.maxSkewDegrees`** (25.0) —
      bench-tune against a physically measured tag placement.
- [ ] **The skew formula's sign** — the formula is a proper pinhole-projection derivation, but
      which physical edge the sensor's corner numbering calls "left" is unknowable without real
      hardware. See the comment in
      [`tag_pose_solver.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/vision/tag_pose_solver.cpp).
- [ ] **Wire it into the competition path** — only after all of the above. See
      [Vision Layer § Wiring it into competition]({{ site.baseurl }}/layers/vision/#wiring-it-into-competition).

---

## 5 · Season-specific game data

Doesn't exist until the game is announced.

- [ ] **[`auton/start_location.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/start_location.hpp)** —
      `kStartLocations` are two labeled placeholders ("Placeholder Start A/B"). Replace with the
      real season's legal starting positions and poses.
- [ ] **[`auton/auton_constants.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/auton/auton_constants.hpp)** —
      `kFieldDimensions` assumes a generic 12 ft × 12 ft field. Confirm.
- [ ] **[`auton/demo_routines.*`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/auton/demo_routines.cpp)** — both routines
      are explicitly placeholder and tied to no real strategy. **Delete or replace** once real
      auton strategy exists.

---

## 6 · Real mechanisms

- [ ] **[`subsystem/demo/example_arm.*`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/subsystem/demo/example_arm.hpp)** —
      the entire class is a reference placeholder ("NOT A REAL MECHANISM"), including its gains,
      its `LOW`/`MID`/`HIGH` presets, and the `kExtendedThresholdRawDegrees` (45°) flag threshold.
- [ ] **Everything referencing `ExampleArm` as a stand-in** needs revisiting once real subsystems
      exist:
  - `AutonomousContext::exampleArm` — swap for real subsystem references
  - The demo button macro in [`demo_macros.cpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/src/lightspeed/driver/demo_macros.cpp)
  - The `exampleArm.isExtended` rule in `kDriveAccelLimitConfig`
  - `Dashboard`'s fault indicator, currently keyed to `ExampleArmState::faulted` specifically

See [Subsystem Layer § Writing your own subsystem]({{ site.baseurl }}/layers/subsystem/#writing-your-own-subsystem).

---

## 7 · Known code cleanups

Not measurements — actual small code changes worth making.

- [ ] **L1 is double-bound** in `opcontrol()`: it runs the intakes (hold) *and* triggers the demo
      arm macro (new press). Rebind the macro once real mechanisms exist. See
      [Driver Control Layer]({{ site.baseurl }}/layers/driver-control/).
- [ ] **Verify the joystick sign convention.** `main.cpp` negates `ANALOG_LEFT_Y` and
      `ANALOG_RIGHT_X` when normalizing. Whether that's correct depends on your motor
      orientation — confirm on the real robot that forward stick means forward robot. See
      [Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/).
- [ ] **`FlagRegistry` has no ownership enforcement.** With one subsystem this is harmless; with
      several, always prefix flag names with the subsystem name. See [Subsystem Layer]({{ site.baseurl }}/layers/subsystem/).
- [ ] **Holonomic kinematics is a stub.** `resolveForwardFallback()` and `resolveStrafeFallback()`
      return 0 for non-tank drivetrains. Only matters if you build a holonomic robot.

---

## Deliberately *not* on this list

Pure software or design decisions with no physical dependency — which drive mode the robot
defaults to, the input curve exponent, the *existence* (as opposed to values) of the vision
gating thresholds, the holonomic fallback stub. Those are documented open decisions, not
outstanding measurements.

---

## Suggested order

```
  1. Ports and motor directions           (§1)     -- nothing works before this
  2. Physical geometry                    (§2)     -- gains can't fix wrong geometry
  3. Verify odometry by hand              --       -- push the robot, watch the pose
  4. Tune the velocity controller         (§3)     -- on blocks
  5. Driver control feel                  (§3)     -- with your actual driver
  6. Tune turn / drive-straight           (§3)     -- on the floor
  7. Tune move-to-pose / pure pursuit     (§3)
  8. Start locations and field dimensions (§5)     -- once the game is announced
  9. Real subsystems                      (§6)
 10. Real autonomous routines             (§5)
 11. Vision, last                         (§4)
```

**Before your first competition,** also do the things no bench audit can substitute for: drive
under real driver control with the accel-limit condition actually toggling, run the selector on
the real screen, and confirm the dashboard and SD log against a live match.

---

**See also:** [Tuning Guide]({{ site.baseurl }}/guides/tuning/) · [Configuration Reference]({{ site.baseurl }}/reference/configuration/) · [Getting Started]({{ site.baseurl }}/getting-started/)
