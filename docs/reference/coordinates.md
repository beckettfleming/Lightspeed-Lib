---
title: Coordinate System
parent: Reference
nav_order: 2
permalink: /reference/coordinates/
---

One convention, used identically by odometry, motion, vision, and the selector GUI. Getting this
wrong is the most common cause of "the robot drove somewhere unexpected", so it's worth five
minutes.

---

## The rules

1. **Distances are inches.** Everywhere above the HAL layer.
2. **Headings are degrees, clockwise-positive**, wrapped to `[0, 360)`.
3. **At heading 0:** robot forward → field **+y**; robot right → field **+x**.

---

## The field frame

```
                       field +y
                          ^
                          |
                          |          heading 0
                          |              ^
                          |              |
                          |          [ ROBOT ]  -->  heading 90
                          |
        ------------------+------------------>  field +x
                          |
                          |
                     (0,0) is the field's own corner
```

Headings increase clockwise when viewed from above:

```
                          0 deg
                            |
                            |
          270 deg ----------+---------- 90 deg
                            |
                            |
                         180 deg
```

| Heading | Robot faces | Driving forward moves it |
|---|---|---|
| 0° | field +y | +y |
| 90° | field +x | +x |
| 180° | field −y | −y |
| 270° | field −x | −x |

---

## Why clockwise-positive?

Because that's what VEXos's inertial sensor reports natively. Rather than flipping the sign at
the HAL boundary and risking one place being missed, the whole codebase adopts the sensor's
convention.

The trade-off is that this is a **left-handed** frame, so the standard rotation formulas look
slightly unusual — note the sign pattern in the fusion core:

```cpp
dx = forward * sin(heading) + strafe * cos(heading);
dy = forward * cos(heading) - strafe * sin(heading);
```

Compare against the more familiar counter-clockwise-positive version, where sin and cos swap
places. If you're porting math in from elsewhere, check this carefully.

`motion::toLocalFrame()` is the exact inverse of this rotation.

---

## Local (robot) frame

`motion::LocalOffset` describes a point relative to the robot:

```cpp
struct LocalOffset {
    double forward;      // ahead of the robot
    double strafeRight;  // to the robot's right
};
```

```
                    forward (+)
                        ^
                        |
                        |
     strafeRight(-) <---+---> strafeRight(+)
                        |
                        |
                    backward (-)
```

Used by `PurePursuitController` and `MoveToPose` to compute steering curvature:

```cpp
curvature = 2 * local.strafeRight / (local.forward^2 + local.strafeRight^2)
```

Positive curvature curves **right**; zero is straight.

---

## Comparing headings — always use the helper

```cpp
double error = motion::headingErrorDegrees(from, to);   // shortest signed error, (-180, 180]
```

Positive means "rotate clockwise to get there".

**Never compare headings by plain subtraction.**

| From | To | Naive `to - from` | `headingErrorDegrees()` |
|---|---|---|---|
| 350° | 10° | **−340°** ✗ | **+20°** ✓ |
| 10° | 350° | **+340°** ✗ | **−20°** ✓ |

A robot using the naive value spins 340° the long way instead of 20° the short way. Every
heading comparison in the codebase goes through the helper.

---

## Continuous vs. wrapped heading

Two representations, for two purposes:

| | Wrapped | Continuous |
|---|---|---|
| Range | `[0, 360)` | unbounded (can be −720 or +1080) |
| From | `Imu::getHeadingDegrees()` | `Imu::getContinuousHeadingDegrees()` |
| Use for | **Display only** | **Any delta math** |

`OdometryFusion` tracks a continuous heading internally and only wraps to `[0, 360)` when
publishing a `Pose`. `IMUSource` reads continuous rotation for the same reason.

If you subtract two wrapped headings across the 0/360 boundary you get a ~359° "delta" instead of
1°, and the pose jumps across the field.

---

## Screen coordinates

The selector GUI maps field inches to screen pixels on the 480 × 272 brain screen.

```cpp
ScreenPoint fieldToScreen(double fieldXInches, double fieldYInches,
                          const FieldToScreenConfig& config, const ScreenRegion& region);
```

- Field `(originXInches, originYInches)` maps to the region's **bottom-left** corner
- Scaled uniformly by `pixelsPerInch` on both axes
- **Field +y maps to screen −y** (upward) — screen y grows downward

```
   Field                      Screen (480 x 272)
                                (0,0) top-left
     +y                             +------------------+
      ^                             |                  |
      |                             |     +y is UP     |
      |                             |     here         |
      +----> +x                     |                  |
   (0,0) bottom-left                +------------------+
                                              y grows down
```

Current mapping: `pixelsPerInch = 1.6`, chosen so a 144 in field fits the 230 × 230 px screen-1
region with margin. Screen 2's preview pane uses the same origin at `1.6 × 0.65`.

---

## Camera frame (vision)

`vision::CameraMountOffset` uses the **robot's local frame**, same convention as
`odom::PodConfig::offsetInches`:

```cpp
struct CameraMountOffset {
    double xInches;               // RIGHT of the tracking center
    double yInches;               // FORWARD of the tracking center
    double headingDegreesOffset;  // camera yaw vs. robot forward, clockwise-positive
};
```

`RelativeTagReading::bearingDegrees` is measured from the camera's optical axis,
clockwise-positive — same handedness as everything else.

`TagWorldPose::pose.headingDegrees` is the direction the tag's **face normal points outward**,
in field coordinates. A tag on the field's west wall facing east into the field has heading 90°.

---

## Pod offsets (if you add tracking wheels)

`odom::PodConfig::offsetInches` is a **signed lever arm from the tracking center**, measured
along the axis **perpendicular to the pod's own rolling direction**:

| Pod role | Offset measures | Positive means |
|---|---|---|
| `forward` | its left/right position | **right** |
| `strafe` | its forward/back position | **forward** |

```
                       forward pod, offset = +6.0
                                 [=]
                                  |
        tracking center  ---------+---------
                                  |
                                 [=]
                       forward pod, offset = -6.0
```

The offset is used for the rotation (chord) correction: the fusion core subtracts
`offsetInches × deltaTheta` from each pod's raw delta to remove the arc the pod swept purely from
the robot rotating.

**Verify the sign empirically**: rotate the robot in place on the bench. If x/y drift while
turning, a sign is wrong — flip it.

---

## Quick sanity checks on real hardware

Do these before trusting any autonomous routine:

| Test | Expected |
|---|---|
| Push the robot **forward** one tile at heading 0 | `odom.pose.y` increases by ~24, `x` unchanged |
| Push it **right** at heading 0 | `x` increases (tank robots can't do this under power, but push it by hand) |
| Rotate **clockwise** 90° | `heading` goes 0 → 90 |
| Rotate **counter-clockwise** 90° | `heading` goes 0 → 270 |
| Rotate in place a full turn | `x` and `y` return to where they started |

That last one is the best single test of your pod offsets and track width. If x/y wander during
a pure rotation, your geometry is wrong.

---

**See also:** [Odometry Layer]({{ site.baseurl }}/layers/odometry/) · [Motion Layer]({{ site.baseurl }}/layers/motion/) · [Troubleshooting]({{ site.baseurl }}/guides/troubleshooting/)
