---
title: Glossary
parent: Reference
nav_order: 4
permalink: /reference/glossary/
---

Terms used throughout Lightspeed and this wiki.

---

## Control theory

**PID / PIDF** — A feedback controller. **P**roportional reacts to current error, **I**ntegral to
accumulated error, **D**erivative to the rate of change. The **F** is feedforward. See
[Control Layer]({{ site.baseurl }}/layers/control/).

**Feedforward** — Output computed from the *target* rather than the error. It predicts what
you'll need before error appears, so feedback only has to correct the leftovers. In Lightspeed:
`kV` (per unit of velocity), `kA` (per unit of acceleration), `kS` (constant, to break static
friction).

**kV** — Feedforward gain: output per unit of target velocity. For the drivetrain, millivolts per
RPM. **The single most important gain to tune** — get it right and PID barely matters.

**kS** — Feedforward gain for static friction, applied as `kS × sign(targetVelocity)`. Fixes a
mechanism that won't start moving on small commands.

**Integral windup** — When the integral term accumulates a large value during a long approach and
then overshoots badly. Guarded here by `integralZone` (only integrate near the target) and
`integralMax` (a hard clamp).

**Slew rate limiting** — Capping how fast a value may change. Prevents jerky motion and wheel
slip. Used on drivetrain voltage (mV/s) and on driver-control velocity targets (RPM/s).

**Settled** — Within tolerance of the target for a number of *consecutive* cycles. Requiring
consecutive cycles rather than one sample prevents declaring victory on a single lucky reading.

**Setpoint** — The value a controller is trying to reach.

**Deadband** — A range of joystick input near center that's treated as zero, so stick drift
doesn't creep the robot.

**Expo curve** — Nonlinear input scaling that compresses the low end for fine control while
keeping full range at the extremes. Controlled by `curveExponent`.

---

## Motion

**Motion profile** — A time-parameterized plan of position, velocity, and acceleration for a
move. Lets the robot follow a smooth planned trajectory instead of slamming toward a target.

**Trapezoidal profile** — Accelerate at a constant rate, cruise, decelerate. Velocity over time
looks like a trapezoid.

**S-curve profile** — A jerk-limited profile where acceleration itself ramps up and down, so
there's no sudden jolt at the start and end. Gentler on the drivetrain. Used when
`maxJerk > 0` **and** the move is long enough to reach both max acceleration and max velocity.

**Jerk** — The rate of change of acceleration (in/s³). Limiting it is what makes an S-curve.

**Curvature** — `1 / turnRadius`, in units of 1/inches. Zero is straight; positive curves right.
Used by pure pursuit and move-to-pose for steering.

**Pure pursuit** — A path-following algorithm: repeatedly pick a point on the path some
**lookahead** distance ahead, and steer toward it along a circular arc.

**Lookahead distance** — How far ahead on the path pure pursuit aims. Too short causes weaving;
too long causes corner cutting. Lightspeed scales it with speed.

**Boomerang / move-to-pose** — Driving to a full `(x, y, heading)` pose in one continuous curve by
chasing a **carrot point** offset behind the target along the *target's* heading.

**Carrot point** — The moving intermediate point a boomerang controller steers toward. It
converges on the target as the robot closes in.

**Waypoint** — A single `(x, y)` point in a path.

**Path smoothing** — Turning a few hand-placed waypoints into a dense, gently curved point list.
Lightspeed uses point **injection** followed by gradient-descent smoothing.

**Track width** — The distance between the left and right wheel contact patches. Directly scales
every turn — a wrong value cannot be fixed by tuning gains.

---

## Odometry

**Odometry** — Continuously estimating the robot's position from sensor data.

**Pose** — Position **and** orientation together: `(x, y, heading)`.

**Dead reckoning** — Estimating position by integrating movement over time. Accurate short-term;
error accumulates, which is what vision correction is for.

**IME** — Integrated Motor Encoder. The encoder built into a V5 motor. Lightspeed uses drive IMEs
as its forward-distance source.

**IMU** — Inertial Measurement Unit. The V5 Inertial Sensor, used here for heading. Two are
averaged when both are ready.

**Tracking wheel / pod** — An unpowered wheel on a Rotation Sensor that measures distance without
the wheel slip a driven wheel suffers. **This robot has none** — the machinery is built but
unused.

**Lever arm (pod offset)** — A pod's signed distance from the tracking center, perpendicular to
its rolling direction. Used to subtract the arc a pod sweeps purely from the robot rotating.

**Tracking center** — The point on the robot whose pose odometry actually reports.

**Chord correction** — Converting the *arc* the robot swept in one cycle into the straight-line
*chord* that belongs in x/y. The factor `2·sin(Δθ/2)/Δθ` approaches 1 as Δθ approaches 0.

**Midpoint heading** — Averaging the heading at the start and end of a cycle before rotating the
local displacement into the field frame. Using the stale start heading biases the pose during
turns.

**Confidence tier** — How much to trust the current pose, based on how much of the configured
sensor topology is healthy: `fullPod` → `partial` → `imeOnly`.

**Continuous vs. wrapped heading** — Wrapped is `[0, 360)` (display); continuous is unbounded
(delta math). Subtracting wrapped headings across 0/360 produces a ~359° error.

---

## Software structure

**HAL** — Hardware Abstraction Layer. The only layer that touches PROS device APIs.

**PROS** — The open-source C/C++ operating system and toolchain for the VEX V5, from Purdue.
Lightspeed is built on PROS 4.2.2.

**Task** — A concurrent thread of execution. Lightspeed runs several (odometry, drivetrain
control, scheduler, GUI, logger, dashboard).

**Mutex** — A lock ensuring only one task touches shared data at a time. A **recursive** mutex
can be re-locked by the same task without deadlocking, which is what makes
`transitionTo()` safe to call from inside `onUpdate()`.

**Singleton** — A class with exactly one instance, reached via `instance()`. Used for
`Scheduler`, `FlagRegistry`, and `TelemetryBus`.

**Blocking** — A call that doesn't return until it finishes. Fine on the autonomous task; never
acceptable in `opcontrol()`.

**Fire and forget** — Starting an action without waiting for it, so something else can happen
concurrently.

**Join point** — Where you *do* wait for a fire-and-forget action to finish
(`waitUntilSettled()`).

---

## Lightspeed-specific

**Subsystem** — A robot mechanism with a state machine, PIDF position control, and named presets.
See [Subsystem Layer]({{ site.baseurl }}/layers/subsystem/).

**Preset** — A named position for a subsystem, e.g. `"LOW"` / `"MID"` / `"HIGH"`.

**Flag** — A named boolean a subsystem publishes so other layers can react without knowing the
subsystem's type. Names should be subsystem-prefixed: `"lift.isExtended"`.

**Accel limit rule** — A mapping from a flag to a maximum acceleration. Every true rule
contributes; the most restrictive wins. Keeps the robot from tipping with a mechanism extended.

**Button macro** — A short scripted sequence bound to a controller button, advanced one step per
tick so it never blocks the driver loop.

**Routine** — One autonomous plan: a name, valid start locations, a preview path, and a run
function.

**Start location** — A named field position. Tapping one on the selector calls
`odometry.setPose()` — which is how the robot knows where it starts.

**Autonomous context** — The bundle of shared drivetrain, odometry, motion, and subsystem
references a routine receives. The same instances driver control uses.

**Telemetry bus** — The named-channel snapshot every layer publishes to and every consumer polls.

**Channel** — One named value on the bus, e.g. `odom.pose.x`.

**Diagnostic mode** — A boot-time service mode (hold **Y**) that replaces normal operation with
serial telemetry streaming and the vision bench test.

**Bucket B** — Shorthand in this repo for "implemented, but needs real robot or season data
before it's true". See [Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/).

---

## VEX specifics

**V5 Brain** — The robot's main controller. 480 × 272 touchscreen, 21 smart ports.

**Smart port** — A numbered port (1–21) for V5 devices. A **negative** port number in
configuration reverses that device.

**Cartridge / gearset** — The gear insert in a V5 motor: red (100 RPM), green (200 RPM), blue
(600 RPM). This robot uses blue on the drive with an external reduction.

**Gear ratio** — Here, **wheel revolutions per motor-output-shaft revolution**. Currently
343/600.

**Omni wheel** — A wheel with rollers on its perimeter that can slide sideways. This robot uses
4-inch drive omnis.

**Tank drive** — Left and right sides driven independently. Cannot strafe — which is why
`resolveStrafeFallback()` returns exactly 0 for tank.

**Holonomic** — A drivetrain that *can* strafe (mecanum, X-drive). Recognized in the code with a
stub fallback path; unused here.

**AprilTag** — A fiducial marker a camera can detect and localize from. Used for pose correction.

**Skew** — How far a tag's face is rotated away from squarely facing the camera. High skew makes
the distance estimate unreliable, which is why it's a rejection gate.

**LVGL** — The graphics library PROS uses for the brain screen. Version 9.2.0 is vendored here.

---

**See also:** [Architecture]({{ site.baseurl }}/architecture/) · [Coordinate System]({{ site.baseurl }}/reference/coordinates/) · [Home]({{ site.baseurl }}/)
