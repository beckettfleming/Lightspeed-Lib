---
title: Layers
nav_order: 4
has_children: true
permalink: /layers/
---


# Layers

Lightspeed is nine layers stacked on top of each other, each depending only on
the ones below it. `hal` talks to hardware. `control` turns targets into
voltages. `odom` figures out where the robot is. `subsystem` runs mechanisms.
`driver` handles the joysticks. `motion` moves the robot to places. `auton`
picks and runs routines. `telemetry` records what happened. `vision` corrects
the pose from AprilTags.

See [Architecture]({{ site.baseurl }}/architecture/) for how they fit together,
including the task model and the rules that hold across all of them.

| Layer | Namespace | Job |
|---|---|---|
| [HAL]({{ site.baseurl }}/layers/hal/) | `lightspeed::hal` | Talk to physical devices |
| [Control]({{ site.baseurl }}/layers/control/) | `lightspeed::control` | Turn targets into voltages |
| [Odometry]({{ site.baseurl }}/layers/odometry/) | `lightspeed::odom` | Track where the robot is |
| [Subsystem]({{ site.baseurl }}/layers/subsystem/) | `lightspeed::subsystem` | Run mechanisms with state machines |
| [Driver Control]({{ site.baseurl }}/layers/driver-control/) | `lightspeed::driver` | Process joystick input |
| [Motion]({{ site.baseurl }}/layers/motion/) | `lightspeed::motion` | Move the robot to a place |
| [Autonomous]({{ site.baseurl }}/layers/autonomous/) | `lightspeed::auton` | Pick and run routines |
| [Telemetry]({{ site.baseurl }}/layers/telemetry/) | `lightspeed::telemetry` | Record and display state |
| [Vision]({{ site.baseurl }}/layers/vision/) | `lightspeed::vision` | Correct the pose from AprilTags |
| [Diagnostics]({{ site.baseurl }}/layers/diagnostics/) | `lightspeed::diagnostics` | Boot-time service mode |
