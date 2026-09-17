# Lightspeed documentation

Welcome. These docs are for anyone using Lightspeed — including people who are
new to PROS, new to this codebase, or new to ideas like odometry and PID.

## Start here

| If you want to… | Read |
| --- | --- |
| Get it working with no explanations | [Easy mode](quick-start.md#easy-mode) |
| Get it working and understand what's going on | [Quick start](quick-start.md) |
| Learn the words used everywhere (pose, heading, PID…) | [Key ideas](concepts.md) |
| See how the parts connect | [Architecture](architecture.md) |
| Set it up on your own robot | [Porting guide](porting.md) |
| Make it drive accurately | [Tuning guide](tuning.md) |
| Find every number that still needs a real value | [Placeholder checklist](placeholders.md) |
| Know what's broken or risky | [Code review (Sept 2026)](code-review.md) |

## Reference: the ten parts

Listed from lowest level (closest to the hardware) to highest.

| Part | Page | In one sentence |
| --- | --- | --- |
| `lightspeed::hal` | [hal](layers/hal.md) | Port numbers, and simple wrappers for motors and sensors. |
| `lightspeed::control` | [control](layers/control.md) | Makes the drive wheels spin at the speed you ask for. |
| `lightspeed::odom` | [odom](layers/odom.md) | Works out where the robot is on the field. |
| `lightspeed::subsystem` | [subsystem](layers/subsystem.md) | A reusable template for arms, lifts, and intakes. |
| `lightspeed::driver` | [driver](layers/driver.md) | Turns joystick input into wheel speeds. |
| `lightspeed::motion` | [motion](layers/motion.md) | Autonomous moves: turn, drive, go to a point, follow a path. |
| `lightspeed::auton` | [auton](layers/auton.md) | Autonomous routines and the touchscreen menu to pick one. |
| `lightspeed::telemetry` | [telemetry](layers/telemetry.md) | Logs data, draws a dashboard, streams data over USB. |
| `lightspeed::vision` | [vision](layers/vision.md) | Uses AprilTags to correct the robot's position (experimental). |
| `lightspeed::diagnostics` | [diagnostics](layers/diagnostics.md) | A bench test mode you get by holding Y at startup. |

## What these docs assume

* You can open a PROS project, build it, and upload it. (If not, the
  [quick start](quick-start.md) walks through it.)
* You've seen basic C++: variables, functions, `struct`, `class`.

You don't need to know control theory. The [key ideas](concepts.md) page
explains what you need.

## A warning about numbers

Most numbers in this codebase are **placeholders**, not tested values. Pages
mark them with ⚠️. Don't trust a number unless the
[placeholder checklist](placeholders.md) says it's been measured or tuned.

## Older material

* [`history/review-pass.md`](history/review-pass.md) — the original review
  report that used to be the README.
* [`history/pre-rewrite-2026-09/`](history/pre-rewrite-2026-09/) — the docs as
  they were before this rewrite.
