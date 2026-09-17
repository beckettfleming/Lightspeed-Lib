# `lightspeed::motion` — autonomous motion

Motion profiling, shared geometry helpers, and five movement primitives. Every
primitive is **blocking**, runs at ~50 Hz on the calling task, has both a settle
condition and a timeout, and ends by commanding
`control::DrivetrainVelocityController::setTargetVelocity()` — never voltage
directly. All of them stop the drivetrain (0, 0) on return, whether they settled
or timed out.

> Blocking means these belong on the **autonomous task**, never in `opcontrol()`.
> For scripted sequences during driver control, use
> [`driver::ButtonMacroRunner`](driver.md#buttonmacrorunner).

## `DrivetrainKinematicsConfig` and conversions

Shared by every primitive.

```cpp
struct DrivetrainKinematicsConfig {
    double trackWidthInches;
    double wheelDiameterInches;
    double gearRatio;   // wheel revs per motor-output-shaft rev
};

struct WheelSpeeds { double leftInchesPerSecond, rightInchesPerSecond; };

double inchesPerSecondToRpm(double inchesPerSecond, const DrivetrainKinematicsConfig&);
double rpmToInchesPerSecond(double rpm, const DrivetrainKinematicsConfig&);

WheelSpeeds curvatureToWheelSpeeds(double forwardInchesPerSecond,
                                   double curvature,             // 1/inches, + = right
                                   double trackWidthInches);

WheelSpeeds angularVelocityToWheelSpeeds(double forwardInchesPerSecond,
                                         double angularVelocityDegPerSecond,  // + = clockwise
                                         double trackWidthInches);
```

This is the boundary where inches/second becomes motor RPM.
`angularVelocityToWheelSpeeds` with `forward = 0` is a turn in place; with a
small angular term it is drive-straight's heading-hold trim.

## `pose_math.hpp`

```cpp
struct LocalOffset { double forward, strafeRight; };

LocalOffset toLocalFrame(const odom::Pose& robotPose, double targetX, double targetY);
double      distanceToPoint(const odom::Pose& robotPose, double targetX, double targetY);
double      headingToPoint(const odom::Pose& robotPose, double targetX, double targetY);
double      headingErrorDegrees(double from, double to);   // wrapped to (-180, 180], + = clockwise
```

`headingErrorDegrees()` is the one to reach for whenever you compare two
headings — it handles wraparound so `355°` to `5°` is `+10°`, not `-350°`.

## `MotionProfile`

Generic **1D time-parameterized** profile: trapezoidal, or jerk-limited S-curve
when `maxJerk > 0` **and** the move is long enough to reach both max
acceleration and max velocity with jerk-limited ramps. Short moves fall back to
trapezoidal automatically, per move.

```cpp
struct MotionProfileConfig {
    double maxVelocity;
    double maxAcceleration;
    double maxJerk = 0.0;   // <= 0 disables S-curve
};

struct MotionState { double position, velocity, acceleration; };

MotionProfile(double startPosition, double endPosition, const MotionProfileConfig& config);

MotionState sample(double timeSeconds) const;   // t clamped to [0, totalDuration]
double      getTotalDuration() const;
```

**Unit-agnostic** — inches for drive-straight, degrees for a future subsystem,
anything for anything. It is not drivetrain-specific; any 1D move can reuse it.

## The five primitives

| Primitive | Signature | Controls | Heading target? |
| --- | --- | --- | --- |
| `TurnToHeading` | `run(double heading)` | Rotation in place | Yes (it *is* the target) |
| `DriveStraightDistance` | `run(double inches)` | Straight line | Holds the starting heading |
| `DriveToPoint` | `turnToPoint(x, y)` / `driveToPoint(x, y)` | Turn then drive | No |
| `MoveToPose` | `run(x, y, heading)` | One continuous curve | Yes |
| `PurePursuitController` | `follow(waypoints)` | Path following | No |

### `TurnToHeading`

A PIDF controller on odometry heading whose output is an **angular velocity
command**, converted to left/right wheel RPM.

```cpp
struct TurnToHeadingConfig {
    control::PIDFConfig pidf;   // settleTolerance in degrees; kV/kA/kS normally 0
    double        timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
};

TurnToHeading(control::DrivetrainVelocityController&, odom::OdometryFusion&,
              const TurnToHeadingConfig&);
void run(double targetHeadingDegrees);   // clockwise-positive
```

Feedforward terms are normally zero here — there is no target velocity for a
static heading target.

### `DriveStraightDistance`

A motion profile supplies the position/velocity/acceleration envelope, a PIDF
tracks odometry forward distance against it, and a **P-only heading trim** holds
the starting heading so the move does not drift off-line.

```cpp
struct DriveStraightDistanceConfig {
    control::PIDFConfig distancePidf;   // settleTolerance in inches
    MotionProfileConfig motionProfile;  // in/s, in/s², in/s³
    double        headingCorrectionKP;  // deg of drift → deg/s of correction
    double        timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
};

void run(double distanceInches);   // negative = backward
```

`distancePidf.kV` should normally be **1.0** — pass the profile's own velocity
straight through as feedforward, with kP/kI/kD providing trim correction only.
The heading held is whatever the pose reads at the *start* of the call.

### `DriveToPoint`

Pure composition — no new control loop.

```cpp
DriveToPoint(TurnToHeading&, DriveStraightDistance&, odom::OdometryFusion&);

void turnToPoint(double targetX, double targetY);    // rotate to face
void driveToPoint(double targetX, double targetY);   // turn, then drive
```

Computes heading and distance from the current pose, then delegates. It is a
**discrete turn-then-drive** with no final heading target — if you need the robot
to arrive facing a particular way, use `MoveToPose`.

### `MoveToPose` (boomerang)

Drives to a full `(x, y, heading)` pose in one continuous curved motion.

```cpp
struct MoveToPoseConfig {
    double leadFraction;               // carrot offset as a fraction (0-1) of remaining distance
    double positionToleranceInches;
    double headingToleranceDegrees;
    std::uint32_t settleCycles;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
    MotionProfileConfig speedProfile;
};

void run(double targetX, double targetY, double targetHeadingDegrees);
```

**How it works.** It steers toward a "carrot" point offset *behind* the target,
along the **target's own heading**, at `leadFraction × remaining distance`. As
the robot closes in, the carrot converges on the target. Because the offset
direction is fixed to the target heading throughout — not the robot's — chasing
the carrot naturally lines the robot up with that heading on arrival, with no
separate heading-blend term. It settles only when **both** position and heading
tolerances are met.

Higher `leadFraction` = earlier, more aggressive turn-in. Lower = straighter
approach with a sharper final turn.

> ⚠️ **Forward-only.** Like pure pursuit, it always drives toward the carrot and
> never reverses, even if the target pose is behind the robot. A target that
> genuinely requires backing up is not handled specially.

### `PurePursuitController`

Adaptive-lookahead path following, with a motion profile providing the speed
envelope over the path's length.

```cpp
struct PurePursuitConfig {
    double minLookaheadInches;
    double maxLookaheadInches;
    double lookaheadSpeedGain;   // lookahead = clamp(min + gain × speed, min, max)
    double positionToleranceInches;
    std::uint32_t settleCycles;
    double timeoutSeconds;
    std::uint32_t loopPeriodMs;
    DrivetrainKinematicsConfig kinematics;
    MotionProfileConfig speedProfile;
    PathSmoothingConfig smoothing;
};

void follow(const std::vector<Waypoint>& rawWaypoints);   // no-op if < 2 points
```

It smooths the raw waypoints first (see below), then follows. Lookahead grows
with speed: short lookahead at low speed tracks tightly, long lookahead at high
speed cuts corners smoothly.

> ⚠️ Also forward-only.

## Path building — `path.hpp`

```cpp
struct Waypoint { double x, y; };

struct PathSmoothingConfig {
    double injectionSpacingInches;     // spacing of injected points along each segment
    double weightData;                 // pull toward original position
    double weightSmooth;               // pull toward neighbors' average
    double smoothingToleranceInches;   // stop when the largest move drops below this
    std::uint32_t maxSmoothingIterations;
};

std::vector<Waypoint> buildSmoothPath(const std::vector<Waypoint>& rawWaypoints,
                                      const PathSmoothingConfig& config);
```

Injects evenly-spaced points along each segment, then iteratively pulls each
interior point toward the average of its neighbors while keeping it near its
originally-injected position. **Endpoints never move.** Returns the input
unchanged if given fewer than two points.

`weightData + weightSmooth` should stay **below 1** or the iteration will not
converge.

This is the standard lightweight alternative to true splines at this scale: a
handful of hand-placed waypoints in, a dense smoothed point list out.

## Constants — `motion_constants.hpp`

```cpp
inline const DrivetrainKinematicsConfig kCherenkovKinematics{
    .trackWidthInches   = 12.0,
    .wheelDiameterInches = 4.0,
    .gearRatio          = 343.0 / 600.0,
};

inline const TurnToHeadingConfig          kTurnToHeadingConfig{ /* kP 0.6, kD 0.05, ±2°, 3 s */ };
inline const DriveStraightDistanceConfig  kDriveStraightDistanceConfig{ /* kP 3.0, kD 0.2, kV 1.0, ±0.5 in, 5 s */ };
inline const MoveToPoseConfig             kMoveToPoseConfig{ /* lead 0.4, ±2 in, ±3°, 6 s */ };
inline const PurePursuitConfig            kPurePursuitConfig{ /* lookahead 6–18 in, 8 s */ };
```

Shared speed envelope across the profiled primitives: `maxVelocity` 40 in/s,
`maxAcceleration` 80 in/s², `maxJerk` 400 in/s³.

> ⚠️ Every gain, tolerance, and the track width are placeholders. `trackWidthInches`
> must be measured, not estimated; `wheelDiameterInches`/`gearRatio` must match
> `odom::kDriveImeConfig`. See [Tuning](../tuning.md) and
> [checklist § 3](../BUCKET_B_CHECKLIST.md).
