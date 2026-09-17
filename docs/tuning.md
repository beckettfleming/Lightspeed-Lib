# Tuning guide

**Nothing in Lightspeed has been tuned on a real robot.** Every gain is a
placeholder. This page tells you what to adjust, in what order, and how to tell
when it's right.

**Follow the steps in order.** Each step depends on the ones before it. If you
tune a turn while the track width is wrong, the gains will quietly make up for
the mistake — then break as soon as you fix the track width.

If words like `kP`, feedforward, or "settled" are unfamiliar, read
[Key ideas § Control](concepts.md#control) first.

## Before you start

* **Keep the terminal open** (`pros terminal`). Lightspeed prints mechanism state
  changes, acceleration-limit changes, a status line every second in driver
  control, and the final position after autonomous.
* **Put a microSD card in the brain.** The logger writes `/usd/lslogNNN.csv`
  with position, target vs. actual speeds, and health, 25 times a second. Graphing
  a log afterwards shows far more than watching the robot.
* **For live graphs** you can use [diagnostic mode](layers/diagnostics.md)
  (hold Y through startup), which streams data over USB. It replaces normal
  driving, so it's only for bench work.
* **Start with the robot on blocks**, wheels in the air, for steps 0 and 1.
* **Know the known issues.** Stall detection may cut power when the robot pushes
  against something, and `MoveToPose`/pure pursuit can stop short. See the
  [code review](code-review.md). Don't try to tune around those — fix them.

## How to write down what you did

When a number stops being a placeholder, replace its `TODO` comment with what you
did, and tick it off in the [placeholder checklist](placeholders.md):

```cpp
.kP = 18.5,   // tuned 2026-09-14 on blocks, starts shaking above ~22
```

The next person — including you, months from now — needs to know which numbers
are real.

---

## Step 0 — Measure the robot

This isn't tuning; it's measuring. Do it carefully. Every later step builds on it.

| What | File | How |
| --- | --- | --- |
| Ports and cartridge color | `hal/config.hpp` | Look at the robot |
| `trackWidthInches` | `motion/motion_constants.hpp` | Measure between left and right wheel contact points |
| `wheelDiameterInches` | `motion_constants.hpp` **and** `odom/odometry_constants.hpp` | Measure the wheel |
| `gearRatio` | same two files | Wheel turns per motor turn. Count gear teeth. |
| `kTrackingWheelDiameterInches` | `odometry_constants.hpp` | Measure the tracking wheel (not the drive wheel) |
| Pod `offsetInches` | `odometry_constants.hpp` | Measure each tracking wheel's distance from the tracking center |

### The speed numbers must agree

The drivetrain controller measures speed in **motor RPM** (the motor's own
output shaft). So:

1. `driver::kMaxDriveRpm` = the cartridge's RPM (600 blue, 200 green, 100 red).
2. `control::kDrivetrainVelocityConfig.pidf.kV` starts at 12000 ÷ that RPM.
3. `gearRatio` in `odometry_constants.hpp` and `motion_constants.hpp` are the
   same number.
4. `wheelDiameterInches` in those two files is the same number.

**Example:** blue motors geared so the wheels spin at 450 RPM.
`kMaxDriveRpm = 600`, `kV` starts at `12000 / 600 = 20`, and both gear ratios
are `450.0 / 600.0`.

> The shipped values use 343 for `kMaxDriveRpm` and 35 for `kV`, which is
> wrong. See [code review #1](code-review.md#1-drivetrain-speed-units-disagree--driver-control-is-capped-at-57-speed).

### Check

* Drive at half stick on blocks. In the terminal status line or SD log, the
  actual RPM should be close to half the cartridge RPM.
* Push the robot forward exactly 24 inches by hand. `y` should read about 24.

---

## Step 1 — Drive speed control

**File:** `include/lightspeed/control/drivetrain_velocity_constants.hpp`
**Robot:** on blocks, wheels in the air.

Every autonomous move works by asking for wheel speeds. If the wheels can't hold
the speed they're asked for, nothing built on top can be accurate. So this comes
first.

Watch `drivetrain.left.targetRpm` and `drivetrain.left.actualRpm` (same for
right) in the SD log.

1. **`kV` first.** Set `kP`, `kI`, `kD`, and `kS` to 0. Start `kV` at
   12000 ÷ cartridge RPM. Ask for a medium speed. Adjust `kV` until the actual
   speed settles close to the target **on its own**. Feedforward should do most
   of the work.
2. **`kS` next.** Ask for a very slow speed, like 20–30 RPM. If the wheels don't
   move, raise `kS` until they just start turning. Too high and the robot jerks
   at low speed.
3. **`kP`.** Raise it until the speed reacts quickly to changes. Back off as soon
   as you see or hear shaking (a buzzing sound at steady speed is a sign).
4. **`kD`.** Add a little if it overshoots. Speed readings are noisy, so this is
   usually small or 0.
5. **`kI`.** Usually leave at 0. If there's a steady gap that `kV` can't fix, add
   a small `kI` and set `integralZone` so it only acts near the target.
   `integralMax` (4000 mV) already limits it.
6. **`maxVoltageSlewRatePerSecond`.** 40000 mV/s means 0 to full power in about
   0.3 s. Lower it if the drive is jerky; raise it if it feels sluggish.

**Check:** jump from stopped to half speed and back. Graph target vs. actual. You
want a quick rise, little overshoot, no wobble, and a small steady gap.

**Then on the ground:** check again. The robot's weight changes things. Later,
check again with a low battery — behavior should stay about the same.

---

## Step 2 — Odometry

**File:** `include/lightspeed/odom/odometry_constants.hpp`

There are no gains here, only measurements. But mistakes here look exactly like
bad tuning later, so test now.

1. **Straight line.** Push the robot forward exactly 24 inches. Check `y`. If it's
   always long or short by the same percentage, the tracking wheel diameter is
   wrong.
2. **Spin.** Turn the robot 360° in place. Heading should return to where it
   started, and `x`/`y` should barely move.
   > **If `x`/`y` move during a spin, a pod's offset is wrong** — usually the
   > sign. This is the most common odometry mistake. Remember: forward pods,
   > positive = left.
3. **Square.** Push the robot around a 4 × 4 ft square back to the start. A few
   inches of error is normal. A foot is not.
4. **Unplug test.** Unplug one tracking wheel while moving. Confidence should drop
   from `FULL_POD` to `PARTIAL` and position should stay sensible. Unplug all of
   them: it should drop to `IME_ONLY` and keep tracking.

---

## Step 3 — Turn to heading

**File:** `motion/motion_constants.hpp` → `kTurnToHeadingConfig`

The output is a **turning speed** in degrees per second. So `kP` means "degrees
per second of turning for each degree the robot is off."

> The placeholder `kP = 0.6` is very low — a 90° turn will probably time out
> before finishing. Expect the real value to be several times higher.

* **`kP`** — raise until turns are quick. Back off if the robot swings back and
  forth around the target.
* **`kD`** — add to reduce overshoot. Turns overshoot more easily than straight
  drives, so this matters more here.
* **`kI`** — leave at 0 unless it always stops a degree or two short.
* **`settleTolerance` / `settleCycles`** — currently within 2° for 8 cycles
  (about 160 ms). Only tighten if the robot can actually hold it; too tight and
  every turn runs until the timeout.
* **`timeoutSeconds`** — longer than your slowest 180° turn, but short enough
  that a stuck robot doesn't waste the whole autonomous period.

**Check:** turn 90°, 180°, and −90°. Each should stop cleanly without hunting.
Do each five times and compare — being consistent matters more than any one
perfect turn.

---

## Step 4 — Drive straight

**File:** `motion_constants.hpp` → `kDriveStraightDistanceConfig`

This move has two jobs: go the right distance, and don't drift sideways.

1. **Speed limits first.** `maxVelocity` (40 in/s), `maxAcceleration`
   (80 in/s²), and `maxJerk` (400 in/s³) should match what the robot can really
   do. Start low. If the robot can't keep up with the plan, the correction terms
   will struggle.
2. **Keep `distancePidf.kV` at 1.0.** This passes the planned speed straight
   through. Don't change it to cover up another problem.
3. **`kP`** — inches/second of correction for each inch behind or ahead. Raise
   for a crisper stop; lower if it wobbles at the end.
4. **`kD`** — smooths the stop.
5. **`headingCorrectionKP`** — keeps it pointed straight. Drive 6 ft and watch
   for sideways drift. Too low: it curves off. Too high: it weaves.

> **Long moves:** the timeout is a fixed 5 seconds. With the default speed limits,
> moves longer than about 170 inches get cut off. Raise `timeoutSeconds` if you
> need long moves ([code review #7](code-review.md#7-moves-longer-than-about-170-inches-get-cut-off)).

**Check:** drive 12 in, 48 in, and backward. Check how far off it stops and how
much it drifts sideways. Repeat 48 in five times and compare.

---

## Step 5 — Move to pose and pure pursuit

Both rely on everything above. Don't start here.

> Both only drive **forward**, and both can stop short of the target and wait
> until their timeout ([code review #4](code-review.md#4-movetopose-and-pure-pursuit-can-stop-short-then-wait-for-the-timeout)).
> If you see that, it's the code, not your gains.

### Move to pose

* **`leadFraction`** (0.4) — the main new setting. Higher = turns in earlier and
  more aggressively. Lower = straighter approach with a sharper turn at the end.
  Test by driving to a spot 4 ft away that needs a 90° change in heading.

### Pure pursuit

* **Lookahead** — `minLookaheadInches` 6, `maxLookaheadInches` 18,
  `lookaheadSpeedGain` 0.3. Short lookahead follows closely but may wobble; long
  lookahead is smoother but cuts corners. Test on a path with a tight corner and
  a long straight.
* **Path smoothing** — `injectionSpacingInches` 2.0, `weightData` 0.5,
  `weightSmooth` 0.25. **Keep `weightData + weightSmooth` below 1.** More
  smoothing means rounder corners and less exact waypoints.

---

## Step 6 — Driver control

**File:** `include/lightspeed/driver/driver_control_constants.hpp`

This is about what the driver likes. **Put your real driver on the controller**
and change one thing at a time.

* **`kDriveMode`** — let them try tank, arcade, and curvature.
* **`curveExponent`** (2.0) — higher = finer control at low speed, but feels
  slower off-center.
* **`deadband`** (0.05) — just above where the sticks rest.
* **`defaultMaxRpmPerSecond`** (2000) — basically no limit. Lower it if the robot
  tips or wheelies when it takes off.
* **Acceleration rules** — one per mechanism that should make driving gentler
  when raised. The terminal prints a line whenever the limit changes, so you can
  confirm the flag is switching.

---

## Step 7 — Mechanisms

Mechanisms use the same PID controller, aimed at a position instead of a speed.
For each one:

* **`kP`** — raise until it reaches its target quickly.
* **`kD`** — add to reduce overshoot. Heavy mechanisms overshoot; add `kD` before
  lowering `kP`.
* **`kI`** — only if gravity keeps it short of the target, and then with an
  `integralZone`.
* **`settleTolerance` / `settleCycles`** — tight enough to matter, loose enough
  to actually reach.
* **Presets** — move the mechanism by hand to each position and read
  `getPositionRaw()`.

**Test the safety stop:** block the mechanism on purpose and confirm it stops.

> A mechanism that has to push hard just to **hold still** (like an arm against
> gravity) may be wrongly flagged as stalled and dropped. That's a known issue
> ([code review #2](code-review.md#2-stall-detection-trips-on-normal-loads)).

---

## Step 8 — Vision

Only after everything above works well. Follow
[vision § Bringing vision online](layers/vision.md#bringing-vision-online). Don't
trust any vision numbers until the bench test matches a tape measure.
