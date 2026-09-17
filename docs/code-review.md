# Code review — September 2026

This is a read-through of the whole codebase (about 6,800 lines of library code
plus `main.cpp`), done while rewriting these docs. It lists what is good, what
is broken or risky, and what to fix first.

**How to read this page.** Each problem has a priority:

* 🔴 **Fix before the first real bench session.** It will stop the robot from
  working correctly, or it will make tuning give you wrong numbers.
* 🟠 **Fix before competition.** It works on a bench but can fail in a match.
* 🟡 **Worth fixing.** Small bugs, rough edges, or misleading comments.

Each problem also says how sure we are:

* **Certain** — follows directly from reading the code.
* **Likely** — very probably true, but depends on how the real hardware behaves.
  Check it on the robot.

Nothing here has been tested on a robot. No code was changed as part of this
review — only the docs.

---

## The short version

The overall design is strong. The layers are cleanly separated, every port and
tunable number lives in one obvious place, and most of the math (odometry, motion
profiles, coordinate conversions) checks out.

The biggest problems are:

1. **The drivetrain speed units don't agree with each other**, so driver control
   is capped at roughly half speed and the velocity feedforward is sized wrong.
2. **Motor stall detection is too twitchy.** It will "fault" a mechanism that is
   simply holding still under load, and cut drive power while pushing.
3. **Two of the motion commands (`MoveToPose` and pure pursuit) can stop short of
   their target and then sit still until they time out.**
4. **A button macro can lock the driver out of the arm buttons** for the rest of
   the match if the mechanism faults.
5. **The auton selector can't be used on a bench without a competition switch**,
   which the old docs didn't mention.

---

## What is good

These parts were checked and hold up:

* **Layer separation.** Lower layers never reach up into higher ones. Only the
  `hal` layer talks to motors and sensors.
* **One set of objects.** `main.cpp` builds each motor group, controller, and
  odometry object exactly once, and both autonomous and driver control use the
  same ones. No two loops fight over the same motors.
* **One place for ports and one place for each tunable.** `hal/config.hpp` holds
  every port. Every gain is in a `*_constants.hpp` file.
* **Odometry math.** The arc-to-chord correction, the midpoint-heading rotation,
  the tracking-wheel lever-arm correction, and the "use two wheels to estimate
  heading" fallback are all correct (with one sign caveat — see below).
* **Motion profile math.** The trapezoid and S-curve formulas were checked by
  hand. Speed and position line up at every phase boundary.
* **Coordinate helpers.** `toLocalFrame`, `headingToPoint`, and the
  curvature/angular-velocity-to-wheel-speed functions all use the same
  clockwise-positive convention consistently.
* **Thread safety.** Shared data is protected by mutexes or atomics in the right
  places. No obvious data races in the normal flow.
* **Safe failure paths.** Output voltages are clamped, every blocking motion
  command has a timeout, the SD logger survives a missing or pulled card, and
  the telemetry bus never allocates memory in fast loops.
* **Comments.** Almost every file explains *why* it is built the way it is.

---

## 🔴 Fix before the first bench session

### 1. Drivetrain speed units disagree — driver control is capped at ~57% speed

**Certain.**

`DrivetrainVelocityController::setTargetVelocity()` compares the target against
`MotorGroup::getVelocityRpm()`, which reads the **motor's own RPM** (0–600 for
blue cartridges). That means every target sent to it is also in motor RPM.

The motion layer gets this right: `inchesPerSecondToRpm()` uses the gear ratio
to turn inches/second into motor RPM.

But two constants assume the target is **wheel RPM** (the 343 RPM after the
external gearing):

| Constant | File | Current value | What it should be for blue motors |
| --- | --- | --- | --- |
| `kMaxDriveRpm` | `driver/driver_control_constants.hpp:29` | 343 | 600 |
| `kV` | `control/drivetrain_velocity_constants.hpp:24` | 35 (12000 ÷ 343) | about 20 (12000 ÷ 600) |

What goes wrong:

* Full joystick asks for 343 motor RPM, which is only 57% of the robot's real
  top speed.
* `kV = 35` means feedforward alone asks for full 12 V at a 343 RPM target, so
  the P term has to fight it back down. Tuning `kP` on top of this will give
  numbers that break once `kV` is fixed.

**Fix:** set `kMaxDriveRpm` to the cartridge RPM (600 for blue, 200 for green,
100 for red) and start `kV` at 12000 ÷ that number. The
[tuning guide](tuning.md) now describes it this way.

### 2. Stall detection trips on normal loads

**Certain** about the logic. **Likely** to matter on the real robot.

`MotorGroup::getHealth()` (`hal/motor_group.cpp:73`) calls a motor "stalled" the
instant it is receiving at least 3 V (25%) while turning slower than 3 RPM.
There is no time delay.

This is also true when nothing is wrong:

* **An arm holding a position against gravity** often needs more than 3 V while
  standing still. The subsystem base class sees "stalled", cuts power, and the
  demo arm goes to `faulted` — so the arm drops.
* **The drivetrain pushing another robot** (or a wall) looks stalled. The
  velocity controller cuts that side to 0 V and resets. Next cycle it's no longer
  "stalled" (voltage is 0), so it ramps back up, trips again, and so on. The
  drive will pulse instead of pushing.
* **Starting from a standstill** under a heavy load may trip it for a cycle.

**Fix:** only report `stalled` after the condition has lasted a while (for
example 300–500 ms), and consider using current draw instead of voltage. Keep
`overTemperature` as an immediate stop.

### 3. One unplugged motor hides every other fault on that side

**Certain.**

`getHealth()` checks for disconnected motors *first* and returns right away
(`hal/motor_group.cpp:58`). If one motor on the left side is unplugged, it
returns `disconnected` every cycle and **never checks the remaining motors for
stalls or overheating**. The drivetrain and subsystem code only stop on
`disconnected` when *all* motors are gone, so the remaining motors will be driven
with no stall or temperature protection.

**Fix:** check the connected motors for over-temperature and stall even when some
are disconnected, or return a count/bitmask instead of one status.

### 4. `MoveToPose` and pure pursuit can stop short, then wait for the timeout

**Certain.**

Both commands get their speed from a motion profile that runs on a **clock**
(`motion/move_to_pose.cpp:52`, `motion/pure_pursuit_controller.cpp:91`). Once the
profile's time is up, the commanded speed is exactly 0 — no matter where the
robot actually is.

* `MoveToPose` sizes its profile using the *straight-line* distance, but the
  robot drives a *curve*, which is longer. It will usually run out of profile
  before arriving.
* Both commands lag behind the profile a little in real life (the robot can't
  track perfectly).

When that happens the robot stops a few inches short, never gets within
tolerance, and sits still until the timeout (6 s and 8 s). That can waste most
of an autonomous period.

**Fix:** base speed on remaining distance (for example, the lower of the
profile speed and a speed that slows down as you approach the end), and keep a
small minimum speed until the robot is inside the tolerance.

### 5. With the placeholder gains, turns will probably time out

**Likely.**

`kTurnToHeadingConfig.kP = 0.6` means 54 °/s of turning when 90° away, and it
slows down as it gets closer. Even with a perfect drivetrain, getting from 90°
off to within 2° takes about 6 seconds, but the timeout is 3 seconds.

This is expected — the gains are placeholders — but it means the demo routines
will look broken until you tune. Expect `kP` to end up several times larger.

---

## 🟠 Fix before competition

### 6. A button macro can lock out the arm buttons

**Certain.**

The demo macro (`driver/demo_macros.cpp:12`) waits until the arm reaches
`holding`. If the arm faults or never settles, that never happens. The macro
stays "running" forever, and `opcontrol()` ignores R1/R2 while a macro runs. The
driver loses the arm buttons until they press L1 again.

`MacroStep` has no timeout.

**Fix:** add a timeout to `MacroStep` (or a cancel button), and make the wait
condition also accept `faulted`/`idle`.

### 7. Moves longer than about 170 inches get cut off

**Certain.**

`DriveStraightDistance` has a fixed 5 s timeout. With the current speed limits
(40 in/s, 80 in/s², 400 in/s³), the profile alone for a 170 in move lasts about
5 s. Anything longer is stopped by the timeout before the profile finishes —
before the robot even gets close. The field diagonal is about 200 in.

The same idea applies to pure pursuit (8 s) on long paths.

**Fix:** make the timeout "profile duration plus a margin" instead of a fixed
number.

### 8. You can't use the auton selector on a bench without a competition switch

**Certain.**

When the brain is not connected to a competition switch or field controller,
PROS runs `opcontrol()` immediately after `initialize()`. The first thing
`opcontrol()` does is stop the selector. So on a plain bench, the selector
disappears as soon as it appears, and `autonomous()` never runs.

This isn't a bug so much as a surprise. The [quick start](quick-start.md)
now explains how to test autonomous.

### 9. The starting pose is set when you *tap*, not when the match starts

**Certain.**

Tapping a start location calls `odometry.setPose()` right then. If anyone moves
the robot afterwards (lining it up, bumping it), odometry tracks that movement
and the robot starts autonomous thinking it's somewhere else.

**Fix:** also re-apply the chosen start pose at the top of `autonomous()`. Until
then: **place the robot first, then tap.**

### 10. Selector taps may be missed

**Likely** — check on hardware.

The selector checks the touchscreen every 50 ms and only acts if the status is
exactly `E_TOUCH_PRESSED` (`auton/selector_gui.cpp:78`). In PROS, "pressed" is
the first moment of a touch; after that it becomes "held". A quick tap can
change state between two checks and be ignored.

**Fix:** act on any new `press_count` whose status is pressed *or* held.

### 11. Stopping the selector mid-action could freeze odometry or the screen

**Likely but rare.**

`SelectorGui::stop()` deletes its task outright (`auton/selector_gui.cpp:36`). If
that happens while the task is in the middle of `odometry.setPose()` (which holds
odometry's lock) or in the middle of drawing (the screen has its own lock), that
lock is never released. Odometry would freeze for the whole match.

It needs a tap at almost the same moment autonomous starts, so it's unlikely —
but the result is very bad.

**Fix:** have the task check a "please stop" flag and exit on its own.

### 12. Curvature drive mode pivots until a quarter stick

**Certain.** Only matters if you switch `kDriveMode` to `curvature`.

The "turn in place" threshold (0.05, `driver/drive_mode.cpp:12`) is applied
*after* the deadband and the squared curve. With the default settings, that
means the robot keeps pivoting in place until the forward stick is past about
26%. At that point, turning suddenly drops to 5% strength.

**Fix:** compare the raw stick value, or lower the threshold to match the curve.

### 13. Diagnostic mode mixes two outputs on one console

**Certain.**

Diagnostic mode starts the serial CSV stream *and* the vision bench test at the
same time (`diagnostics/diagnostic_mode.cpp:30`). Both print to the same USB
console at 10 Hz, so their lines interleave. A plotting script reading the CSV
will choke on the vision lines, and the vision output is hard to read.

**Fix:** pick one per boot (for example Y = vision test, X = serial stream).

### 14. Diagnostic mode needs Y held for 5+ seconds

**Certain.**

The Y button is checked near the end of `initialize()`, after both IMUs finish
calibrating (`src/main.cpp:217`). You have to keep holding Y through that whole
wait, not just "at boot". The docs now say so.

---

## 🟡 Worth fixing

### 15. Tracking pod offset sign — the code comment is wrong

**Certain.**

`types.hpp:44` says a forward pod's positive offset means "right". But the math
in `odometry_fusion.cpp:191` only cancels rotation correctly if **positive means
left**, which is what `odometry_constants.hpp` uses (left pod = +6). For strafe
pods, positive = forward, as documented.

The docs now say "forward pods: positive = left". Fix the comment in `types.hpp`
to match. Still do the spin test on the real robot, because a reversed sensor
can flip it again.

### 16. More than 4 pods of one kind overwrites memory

**Certain.**

The `OdometryFusion` constructor (`odometry_fusion.cpp:35`) writes pods into a
fixed array of 4 per role without checking. A fifth forward pod writes past the
end of the array. The old docs said extra items are "dropped, not crashed" —
that's true for telemetry, flags, and subsystems, but not pods.

### 17. Creating a subsystem after the scheduler starts can crash

**Likely but rare.**

`Subsystem`'s constructor registers itself with the scheduler
(`subsystem/subsystem.hpp:58`) before the child class has finished being built.
If the scheduler is already running, it can call `update()` on the half-built
object, which calls a pure virtual function and crashes. The old docs said
registering after `start()` is safe. **Always create subsystems before calling
`Scheduler::instance().start()`.**

### 18. One unplugged drive motor turns off that side's encoder input

**Certain.**

`IMESource::isHealthy()` (`odom/ime_source.cpp:12`) is false if *any* motor on
the side is unplugged, even though the position average already skips the dead
motor. With zero tracking pods, one loose cable stops forward tracking on that
side.

### 19. Battery compensation turns output *down* on a full battery, and jumps at 11 V

**Certain.**

Output is scaled by 12.0 V ÷ battery voltage. A fresh V5 battery reads above
12 V, so output is scaled *down* slightly (for example ×0.94 at 12.8 V). Below
11 V the scale snaps from about ×1.09 back to ×1.0, which is a sudden 9% drop
in power. Consider a smooth fade instead of a step.

### 20. The dual-IMU heading jumps if one IMU drops out

**Certain.**

With two IMUs, heading is their average. If one stops reporting, heading
switches to the other one, and the difference between them shows up as a
sudden heading change (`odom/imu_source.cpp:16`).

### 21. Odometry velocity is probably jittery

**Likely.**

Odometry runs every 5 ms, but V5 motors, rotation sensors, and IMUs normally
update about every 10 ms. Many cycles see no change, then a double-size change,
so `getVelocity()` will bounce. Pure pursuit uses that speed for its lookahead
distance. Consider smoothing velocity or matching the sensor rate.

### 22. The "faulted" state only lasts one cycle

**Certain.**

In the demo arm, `faulted` sets 0 V and goes back to `idle` once health reads
`ok`. With 0 V the stall check always reads `ok`, so it leaves `faulted` on the
very next cycle (20 ms). The dashboard's red fault box will basically never be
seen.

### 23. Vision distance gets less accurate away from image center

**Likely.**

`tagSize × focalLength ÷ pixelSize` (`vision/tag_pose_solver.cpp:43`) gives
depth straight ahead of the camera, but the code treats it as distance along the
line to the tag. The error grows as the tag moves toward the edge of the image
(about 6% at the edge with the placeholder calibration). Dividing by
`cos(bearing)` fixes it.

### 24. Smaller things

* The auton log file may be missing the `auton.routineName` column, because
  that channel is created at the same moment the file opens.
* `headingErrorDegrees()` returns values in [-180, 180), not (-180, 180] as the
  old docs said. Only matters at exactly 180°.
* Arcade drive clips each side instead of scaling both down, so at full forward
  plus turn the robot turns less than expected.
* The route previews for the demo routines are drawn from Start A even when
  Start B is chosen.
* `BUCKET_B_CHECKLIST.md` referred to a "November 2026" review, which hasn't
  happened yet — probably meant an earlier month.
* The last build in `bin/` is from August 27, but several source files were
  changed after that. The "compiles with zero warnings" claim should be
  re-checked with `pros build`. (It couldn't be checked during this review.)
* Most of `docs/` was never committed to git. Commit it so this work isn't lost.

---

## Suggested order

1. Fix the speed units (#1) — it's a two-number change.
2. Add a time delay to stall detection and fix the disconnect masking (#2, #3).
3. Fix the pod offset comment (#15) so nobody "corrects" the right numbers.
4. Do the bench bring-up in the [quick start](quick-start.md) and
   [tuning guide](tuning.md).
5. Fix `MoveToPose` / pure pursuit stopping short (#4) and the fixed timeouts (#7)
   before relying on them in autonomous.
6. Add a macro timeout (#6) and re-apply the start pose in `autonomous()` (#9).
