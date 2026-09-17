# Key ideas

This page explains the words and ideas used throughout Lightspeed. If a doc page
uses a term you don't know, look here first.

---

## The field and the robot's position

### Pose

A robot's **pose** is where it is and which way it's facing:

* `x` — inches to the right from the field's corner
* `y` — inches forward from the field's corner
* `heading` — which way the robot is pointing, in degrees

### Field coordinates

Picture looking down at the field from above:

```
   +y (forward at heading 0)
    ▲
    │
    │        heading 0   = facing +y (up)
    │        heading 90  = facing +x (right)
    │        heading 180 = facing −y (down)
    │        heading 270 = facing −x (left)
    │
  (0,0) ─────────────────►  +x
```

* **Heading goes clockwise.** Turning right makes the number bigger. This
  matches how the V5 inertial sensor reports it.
* Headings wrap around: after 359° comes 0°.
* A standard field is 144 × 144 inches (12 × 12 feet).

### Heading error

The shortest turn from one heading to another. From 350° to 10° is **+20°**
(turn right 20°), not −340°. Lightspeed's `headingErrorDegrees()` does this
for you.

### Odometry

**Odometry** means working out the robot's position by adding up lots of small
movements. Every 5 ms, Lightspeed asks "how far did each wheel roll, and how much
did the robot turn?" and adds that to the last known position.

It's very accurate over short times but slowly drifts, because small errors add
up. That's why you set the starting position at the start of every match.

### Confidence tier

How much odometry trusts its current position, based on which sensors are
working:

| Tier | Means |
| --- | --- |
| `FULL_POD` | All tracking wheels are working. Best accuracy. |
| `PARTIAL` | Some tracking wheels stopped reporting. |
| `IME_ONLY` | No tracking wheels — using the drive motors' encoders. Least accurate, because drive wheels slip. |

---

## Sensors and motors

### Port

A numbered socket on the V5 Brain (1–21). In Lightspeed, a **negative** port
number means "reverse this device."

### Motor cartridge

The gear insert inside a V5 motor that sets its top speed: **red** 100 RPM,
**green** 200 RPM, **blue** 600 RPM.

### RPM

Revolutions per minute. In Lightspeed, drivetrain speeds are in **motor RPM** —
the speed of the motor's output shaft, *before* any extra gears you add.

### Gear ratio

In Lightspeed, `gearRatio` = **wheel turns per motor turn**. If the motor spins
600 RPM and the wheel spins 450 RPM, the ratio is `450.0 / 600.0 = 0.75`.

### Millivolts (mV)

How much power is sent to a motor. **12000 mV is full power**, 0 is off, and
negative values run it backwards.

### IME (integrated motor encoder)

The position sensor built into every V5 motor. Lightspeed can use drive motor
encoders to measure distance, but drive wheels can slip, so they're the backup.

### IMU (inertial sensor)

The V5 Inertial Sensor. Lightspeed uses it to know which way the robot is
facing. It must **calibrate for a few seconds without moving** at startup.
Lightspeed supports one or two.

### Tracking wheel (pod)

A small unpowered wheel that rolls along the floor with a Rotation sensor on it.
Because it isn't driven, it doesn't slip, so it measures distance more accurately
than drive wheels.

* A **forward** pod rolls when the robot drives forward or backward.
* A **strafe** pod is turned sideways and rolls when the robot slides sideways
  (for example, when it gets pushed).

Lightspeed supports 0–4 of each.

### Tracking center

The point on the robot that odometry tracks. Pod offsets are measured from here.
Usually this is the point the robot spins around when turning in place.

### Offset (lever arm)

How far a tracking wheel is from the tracking center, sideways to the direction
it rolls. When the robot spins in place, a pod that isn't at the center still
rolls — odometry uses the offset to subtract that out.

**In Lightspeed:** for forward pods, **positive = left** of center. For strafe
pods, **positive = forward** of center.

---

## Control

### Setpoint / target

The value you want to reach — a speed, an angle, or a position.

### Error

Target minus what the sensor says. If you want 200 RPM and the motor is at
180 RPM, the error is 20.

### PID

A standard way to get to a target by looking at the error:

* **P (proportional)** — push harder the farther away you are. `kP` sets how
  hard. Too high and it overshoots and shakes.
* **I (integral)** — push harder the *longer* you've been off target. Fixes
  small steady errors. Can cause overshoot ("windup"), so it's usually 0.
* **D (derivative)** — push back if you're approaching the target quickly.
  Reduces overshoot.

### Feedforward

A good first guess at the output, *before* looking at error. If you know it
takes about 20 mV per RPM to spin a motor, send that straight away, and let PID
fix the rest.

* **`kV`** — output per unit of target speed.
* **`kS`** — a small fixed boost to get past friction when starting to move.
* **`kA`** — output per unit of target acceleration (usually 0 here).

"**PIDF**" means PID plus feedforward.

### Settled

A controller is **settled** when the error has stayed within a tolerance
(`settleTolerance`) for several cycles in a row (`settleCycles`). Needing
several cycles stops a quick swing through the target from counting.

### Timeout

Every autonomous move gives up after a set time, so a stuck robot doesn't waste
the whole autonomous period.

### Slew rate limit (ramping)

A cap on how fast a value can change. Instead of jumping from 0 to full speed,
it ramps up. Lightspeed uses this in two places: on drive voltage, and on driver
speed while a mechanism is raised.

---

## Driving

### Deadband

Joysticks rarely rest at exactly 0. The deadband ignores tiny stick movement so
the robot doesn't creep.

### Input curve

Makes small stick movements gentler without lowering top speed. A curve exponent
of 1 is straight-line; 2 means half stick gives about quarter speed.

### Drive modes

* **Tank** — left stick drives the left side, right stick drives the right side.
* **Arcade** — one stick for forward/back, the other for turning.
* **Curvature** — like arcade, but turning gets gentler at high speed. At a
  standstill it turns in place.

### Acceleration limit

How fast the driver's speed may change, in RPM per second. Lightspeed makes this
stricter while a mechanism is raised, so the robot is less likely to tip.

---

## Autonomous motion

### Motion profile

A plan for how speed should change over a move: speed up, cruise, slow down.
Following a plan is smoother and more repeatable than "go full speed until close."

* **Trapezoid** — speed ramps up at a constant rate, cruises, ramps down.
  Graphed, it looks like a trapezoid.
* **S-curve** — also limits how quickly the *acceleration* changes (called
  **jerk**), so starts and stops are gentler.

### Curvature (of a path)

How sharply the robot is turning. 0 is straight; bigger numbers are tighter
turns.

### Pure pursuit

A way to follow a path: pick a point a little way ahead on the path (the
**lookahead point**) and steer toward it. Repeat many times per second. A short
lookahead follows the path closely but can wobble; a long one is smoother but
cuts corners.

### Boomerang (move to pose)

A way to drive to a point *and* end up facing a chosen direction in one smooth
curve. The robot steers toward a "carrot" point placed behind the target along
the final heading. As it gets closer, the carrot slides onto the target.

### Waypoint

A point on a path you want the robot to pass through.

---

## Program structure

### Task

A PROS task is a piece of code that runs at the same time as other code. Lightspeed
runs several tasks at once: odometry, drive control, mechanisms, logging, and more.

### Blocking vs. non-blocking

* A **blocking** function doesn't return until it's done. `turnToHeading.run(90)`
  waits until the turn finishes. Fine in autonomous.
* A **non-blocking** function returns right away and the work happens in the
  background. Driver control must only use non-blocking code, or the driver loses
  control while it waits.

### Mutex

A lock that stops two tasks from changing the same data at the same time.

### Subsystem

A mechanism (arm, lift, intake) with its own motors. In Lightspeed, each one is a
class built from the `Subsystem` template.

### State machine

A way of organizing a mechanism's behavior as a set of named **states** (for
example `idle`, `movingToTarget`, `holding`, `faulted`) with clear rules for
switching between them.

### Preset

A named position for a mechanism, like `"LOW"` or `"HIGH"`.

### Flag

A named true/false value a subsystem publishes, like `"lift.isExtended"`. Other
code (such as driver control) can read it by name.

### Telemetry

Data the robot reports about itself: position, speed, motor health, and so on.

### Placeholder

A number in the code that is a guess, waiting for a real measurement or tuning.
See the [placeholder checklist](placeholders.md).

---

## Vision

### AprilTag

A black-and-white square pattern, like a simple QR code, that a camera can find
and identify. If you know where a tag is on the field, seeing it tells you where
the robot is.

### AI Vision Sensor

The VEX camera that can detect AprilTags.

### Calibration (camera)

Numbers describing the camera's lens, like its focal length. Needed to turn a
tag's size in pixels into a distance in inches.
