# Placeholder checklist

Everything on this list **works in code**, but uses a made-up value because the
real one needs a measurement, the real robot, or information about this season's
game.

To finish an item: change the named value in the named file, write what you did
in the comment next to it, and tick the box here.

> This used to be called `BUCKET_B_CHECKLIST.md`.

---

## 1. Ports and motors

All in [`include/lightspeed/hal/config.hpp`](../include/lightspeed/hal/config.hpp).

- [ ] **Drive motor ports** — `kLeftDriveFront/Middle/Back`,
  `kRightDriveFront/Middle/Back`. Set once the drivetrain is wired.
- [ ] **Tracking wheel and IMU ports** — `kLeftForwardPodRotation`,
  `kRightForwardPodRotation`, `kStrafePodRotation`, `kPrimaryImu`,
  `kSecondaryImu`. The placeholder assumes 2 forward pods, 1 strafe pod, and 2
  IMUs. If yours is different, also update `odometry_constants.hpp` and
  `main.cpp` (see the [porting guide](porting.md#step-3--tracking-wheels-and-inertial-sensors)).
- [ ] **Demo arm port** — `kExampleArmMotor`. Reuse or remove when a real
  mechanism replaces the demo arm (see §6).
- [ ] **AI Vision port** — `kAiVisionSensor`. Set once the camera is mounted.
- [ ] **AprilTag family** — `kAiVisionSensor`'s `tag_16H5` is a guess. Set it to
  the family the season's field uses.
- [ ] **Drive cartridge** — `kLeftDriveGroup` / `kRightDriveGroup`. The
  placeholder assumes blue (600 RPM) motors with extra gearing down to about
  343 RPM at the wheel.

## 2. Odometry measurements

All in [`include/lightspeed/odom/odometry_constants.hpp`](../include/lightspeed/odom/odometry_constants.hpp).

- [ ] **Tracking wheel size** — `kTrackingWheelDiameterInches` (currently 2 in).
  Measure the real tracking wheel; it's not the 4 in drive wheel.
- [ ] **Pod offsets** — `kCherenkovTopology.pods[*].offsetInches` (left forward
  +6, right forward −6, strafe 0). Measure each once mounted. Forward pods:
  positive = left. Strafe pods: positive = forward.
- [ ] **Drive gear ratio** — `kDriveImeConfig.gearRatio` (343/600). Confirm the
  real gearing.

## 3. Drive and motion gains

These all need tuning on the real robot. Nothing can be worked out without its
weight, friction, and battery. Follow the [tuning guide](tuning.md).

- [ ] [`control/drivetrain_velocity_constants.hpp`](../include/lightspeed/control/drivetrain_velocity_constants.hpp)
  — `kP`, `kI`, `kD`, `kV`, `kS`, `maxVoltageSlewRatePerSecond`.
  **`kV` is currently sized for the wrong unit** — start it at 12000 ÷ cartridge
  RPM. (`lowBatteryMillivolts` = 11 V is a normal V5 value, not a placeholder.)
- [ ] [`motion/motion_constants.hpp`](../include/lightspeed/motion/motion_constants.hpp)
  — `kCherenkovKinematics.trackWidthInches`, and every gain, tolerance, and
  timeout in `kTurnToHeadingConfig`, `kDriveStraightDistanceConfig`,
  `kMoveToPoseConfig` (including `leadFraction`), and `kPurePursuitConfig`.
- [ ] [`driver/driver_control_constants.hpp`](../include/lightspeed/driver/driver_control_constants.hpp)
  — `kMaxDriveRpm` (**should be the cartridge RPM, not 343** — see
  [code review #1](code-review.md#1-drivetrain-speed-units-disagree--driver-control-is-capped-at-57-speed))
  and `kDriveAccelLimitConfig.defaultMaxRpmPerSecond`.

## 4. AprilTag vision

All in [`include/lightspeed/vision/vision_constants.hpp`](../include/lightspeed/vision/vision_constants.hpp).
None of this is used in matches, because all of it is made up.

- [ ] **Camera calibration** — `kAiVisionCalibration` (focal length, image
  center, printed tag size). Get the real sensor's calibration and measure your
  printed tag.
- [ ] **Camera position** — `kAiVisionMountOffset`. Measure once mounted.
- [ ] **Tag map** — `kTagWorldMap`. Replace with the season's real AprilTag IDs
  and field positions.
- [ ] **Gating** — `kVisionGatingConfig.stableFrameThreshold` and
  `.maxSkewDegrees`. Tune using the vision bench test in
  [diagnostic mode](layers/diagnostics.md).
- [ ] **Skew sign** — whether a positive skew means the tag is turned left or
  right depends on how the AI Vision sensor numbers the tag's corners. Check
  with a real sensor; see `tag_pose_solver.cpp`.

## 5. This season's game

- [ ] [`auton/start_location.hpp`](../include/lightspeed/auton/start_location.hpp)
  — `kStartLocations` has two fake entries ("Placeholder Start A/B"). Replace with
  the real legal starting spots.
- [ ] [`auton/auton_constants.hpp`](../include/lightspeed/auton/auton_constants.hpp)
  — `kFieldDimensions` assumes 12 × 12 ft. Confirm.
- [ ] [`auton/demo_routines.cpp`](../src/lightspeed/auton/demo_routines.cpp)
  — both routines are demos, not strategy. Replace.

## 6. Real mechanisms

- [ ] [`subsystem/demo/example_arm.cpp`](../src/lightspeed/subsystem/demo/example_arm.cpp)
  — the whole demo arm (gains, presets, and the "extended" threshold) is an
  example, not a real mechanism.
- [ ] Everything that uses the demo arm needs updating when it goes:
  `AutonomousContext::exampleArm`, the demo macro (`demo_macros.cpp`), the
  `exampleArm.isExtended` rule in `kDriveAccelLimitConfig`, the dashboard, and
  the R1/R2/L1 buttons in `opcontrol()`.
- [ ] The dashboard title says "Cherenkov" (`telemetry/dashboard.cpp`). Change it
  if your robot has a different name.

---

**Not on this list on purpose:** choices that don't need a measurement, such as
which drive mode to use or the joystick curve. Those are up to your team.
