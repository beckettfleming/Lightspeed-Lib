---
title: Getting Started
nav_order: 2
permalink: /getting-started/
---

## What you need

| Requirement | Notes |
|---|---|
| **PROS CLI** | Version 3.x. Install from [pros.cs.purdue.edu](https://pros.cs.purdue.edu/v5/getting-started/) |
| **ARM GNU Toolchain** | 14.2.1 is what this project has been verified against. The PROS installer bundles a toolchain. |
| **A VEX V5 Brain** | Plus a controller and a USB or wireless connection |
| **An editor** | VS Code with the PROS extension is the usual setup; the repo ships a `.vscode/c_cpp_properties.json` |

The PROS kernel (4.2.2) and LVGL (9.2.0) are already vendored in `firmware/` and
`include/`, so there is nothing to fetch.

---

## Build

From the repo root:

```bash
pros make        # or just `make`
```

A clean rebuild:

```bash
pros make clean
pros make
```

The build should finish with **zero warnings and zero errors** under `-Wall -Wextra`. If you
see warnings, something you changed introduced them — this project keeps a clean build as a
hard rule.

Output lands in `bin/`:

- `bin/hot.package.bin` — your code (fast to re-upload)
- `bin/cold.package.bin` — the PROS kernel and libraries (rarely changes)

Hot/cold linking is enabled (`USE_PACKAGE := 1` in the `Makefile`), which is why re-uploading
after a small change is quick.

---

## Upload and run

```bash
pros upload      # push to the brain
pros terminal    # watch the console output
```

Or in one step:

```bash
pros mu          # make + upload, then open the terminal
```

The console is genuinely useful in this project — the drivetrain, motion primitives, subsystem
state machine, SD logger, and auton selector all print progress lines. See
**[Telemetry Layer]({{ site.baseurl }}/layers/telemetry/)** for what gets printed where.

---

## What happens on boot

```
   Brain powers on
        │
        ▼
  initialize()  ─────────────────────────────────────────────┐
        │                                                    │
        │  1. Construct motor groups, IMUs, drivetrain       │
        │  2. Calibrate both IMUs (BLOCKING, ~2-3 seconds)   │
        │  3. Build odometry fusion (starts its 200Hz task)  │
        │  4. Build all five motion primitives               │
        │  5. Build the demo subsystem + start the scheduler │
        │  6. Build driver-control objects                   │
        │  7. Register autonomous routines                   │
        │  8. Build the vision + serial objects              │
        │                                                    │
        ▼                                                    │
  Is the Y button held on the controller? ───── yes ─────────┤
        │ no                                                 ▼
        │                                          DIAGNOSTIC MODE
        │                                          (never returns —
        │                                           see [Diagnostics Mode]({{ site.baseurl }}/layers/diagnostics/))
        ▼
  9. Start the auton selector GUI (touchscreen)
  10. Start the SD logger and dashboard tasks
        │
        ▼
  ┌─────────────────────────────────────────┐
  │  Driver taps a start location, then a   │
  │  routine, then Confirm — any time       │
  │  before the match begins                │
  └─────────────────────────────────────────┘
        │
        ▼
  Field control starts the match
        │
        ├──► autonomous()  — stops the GUI, runs the selected routine
        │
        └──► opcontrol()   — stops the GUI, runs the driver loop at 100Hz
```

**The IMU calibration in step 2 blocks for 2–3 seconds.** That is expected and is why it lives
in `initialize()`. Do not move the robot during it.

---

## Your first five minutes on real hardware

Do these in order. Each one depends on the previous working.

1. **Fix the ports.** Open [`include/lightspeed/hal/config.hpp`](https://github.com/beckettfleming/Lightspeed-Lib/blob/main/include/lightspeed/hal/config.hpp)
   and set every port number to match your actual wiring. Nothing else works until this is right.
   See **[HAL Layer]({{ site.baseurl }}/layers/hal/)**.
2. **Verify the drivetrain direction.** Push the left stick forward. If the robot goes backward
   or spins, you have a sign problem — see **[Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/)**.
3. **Verify odometry.** Push the robot forward by hand one tile (24 in) and watch `odom.pose.y`
   on the console or dashboard. Then rotate it 90° and watch the heading.
4. **Measure and enter the physical geometry** — track width, wheel diameter, gear ratio.
   See **[Setup Checklist]({{ site.baseurl }}/guides/setup-checklist/)**.
5. **Tune the velocity controller** with the wheels off the ground. See **[Tuning Guide]({{ site.baseurl }}/guides/tuning/)**.

Only after all five should you try an autonomous routine.

---

## Repository layout

```
Lightspeed-Lib/
├── include/lightspeed/     ← all library headers, one folder per layer
│   ├── hal/                   hardware wrappers + THE PORT MAP
│   ├── control/               PIDF, slew limiter, drivetrain velocity
│   ├── odom/                  odometry sources + fusion
│   ├── subsystem/             mechanism framework + scheduler
│   ├── driver/                joystick processing, drive modes, macros
│   ├── motion/                motion primitives + path following
│   ├── auton/                 routines, registry, touchscreen selector
│   ├── telemetry/             data bus, SD logger, dashboard, serial
│   ├── vision/                AprilTag pose correction
│   └── diagnostics/           boot-time service mode
├── src/
│   ├── lightspeed/         ← implementations, mirroring include/
│   └── main.cpp            ← PROS entry points; builds every shared object
├── include/pros/, include/liblvgl/, firmware/   ← vendored PROS 4.2.2 + LVGL 9.2.0
├── docs/BUCKET_B_CHECKLIST.md
├── Makefile, common.mk, project.pros
└── wiki/                   ← this documentation
```

Every header has a substantial file-level comment explaining *why* it exists and what design
decision it encodes. Those comments are the authoritative source; this wiki summarizes them.
