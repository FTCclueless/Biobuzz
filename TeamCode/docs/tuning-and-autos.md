# Tuning and autos: the practical guide

Two questions answered here: **how do I write an auto**, and **what do I tune, with which
tester, and where does the number go**.

Driving and pathing use **Pedro Pathing 3**. Every drive constant lives in ONE file:
`pedroPathing/Constants.java`. Mechanism constants (flywheel, turret, intake) live in their
own classes and show up in FTC Dashboard.

---

## Part 1 — Writing an auto

### Yes, it really is just x/y points

```java
import static com.pedropathing.api.Paths.*;

Path toGoal = line(new Pose(12, 60), new Pose(48, 60)).constant(0);
```

Field coordinates, **inches** and **radians**, heading 0 along +x, counter-clockwise positive.
These are the same coordinates `ROBOT_POSITION` and `BiobuzzField` use.

### Running it

```java
robot.drivetrain.follow(toGoal);
robot.waitWhile(robot.drivetrain::isBusy);
```

`ExampleAuto` wraps that in a `follow()` helper. Copy it. `robot.update()` runs the follower
every loop, so the shooter, intake and turret keep working while the robot drives.

### Shapes and heading

| Code | Shape |
|---|---|
| `line(a, b)` | Straight line |
| `curve(a, c1, ..., b)` | Bezier curve; the middle points pull the curve toward them |
| `through(a, p1, ..., b)` | Smooth curve that passes through every point |
| `path(p1, p2, ...)` | Chain several paths into one, without stopping between them |

Then pick a heading for the path:

| Code | What the robot faces |
|---|---|
| `.constant(h)` | Always `h` |
| `.linear(h0, h1)` | Turns from `h0` to `h1` along the path |
| `.tangent()` | Along the path |
| `.reverseTangent()` | Backwards along the path |
| `.facingPoint(pose)` | Always at a point, e.g. the goal |

### Rules

- Build paths in `init`, not in the loop.
- Set the starting pose to the real one: `robot.drivetrain.setPoseEstimate(startPose)`.
- To drive to a point and hold it with no path, use `robot.drivetrain.goToPoint(pose)`.

---

## Part 2 — What to tune, in order

Run these in order. Each one depends on the one before it. The Pedro tuners are TeleOp
opmodes in groups **1** to **4**. They build the follower straight from `Constants`, so the
shooter and intake stay off.

### Step 1 — Pod offsets (group "1")

Set both offsets in `pinpointConfig` to `0`, then run **AutomaticOffsetsTuner** (spins in
place) or **OffsetsTuner** (you turn it by hand).

| Telemetry | Goes into |
|---|---|
| `forwardY` | `pinpointConfig.xPodOffset` |
| `strafeX` | `pinpointConfig.yPodOffset` |

Then run **LocalizationTest**: drive a lap and come back to a mark. Under about 1 inch and
2 degrees of error is good. Nothing else works until this does.

### Step 2 — Identification (group "2")

| Tuner | Gives you | Goes into `foresightConfig.` |
|---|---|---|
| ForwardVelocityIdentification | Forward Velocity | `maxAchievableForwardVelocity` |
| StrafeVelocityIdentification | Max Achievable Strafe Velocity | `maxAchievableStrafeVelocity` |
| ForwardDecelerationIdentification | Forward Max Achievable Deceleration | `naturalForwardDeceleration` |
| StrafeDecelerationIdentification | Max Achievable Strafe Deceleration | `naturalStrafeDeceleration` |
| ForwardBrakingIdentification | Forward Linear / Quadratic Brake Coefficient | `linearBrakeCoefficients`, `quadraticBrakeCoefficients` |

The brake coefficients are a diagonal matrix, `Matrix.diag(forward, strafe)`. There is no
strafe braking tuner, so both entries use the forward numbers for now.

### Step 3 — Controllers (group "3")

**HeadingAutoTuner**, **ForwardTranslationalAutoTuner** and **StrafeTranslationalAutoTuner**
each run in place and print a "Large" and a "Small" set of coefficients. They go into
`headingFeedback`, `forwardTranslational` and `strafeTranslational`. The heading tuner's
"Heading Feedforward" goes into `headingStaticFF`. Run these after Step 2, because the
translational tuners use the brake coefficients.

### Step 4 — Path tests (group "4")

**LineTest**, **StrafeLineTest**, **Curves** and **CompoundCurve** drive back and forth
forever. Watch `TranslationalError` and the robot on the Dashboard field. If they drift,
go back to Step 1, not to the gains.

### TeleOp heading lock

`Drivetrain.headingPID` and `Drivetrain.HEADING_K_STATIC` (Dashboard → Drivetrain) hold the
heading when TeleOp locks onto the goal. Tune them by hand.

### Steps 5–6 the fast way — Flywheel Auto Tuner

**Flywheel Auto Tuner** does the flywheel's whole characterize → tune → verify sequence in one
run (about 2 minutes), with no pasting in between:

1. **Characterize** — measures `kV`/`kA`/`kS` and `FF_TABLE`, and applies them live.
2. **Sweep** — retries the closed loop at progressively stiffer `MAX_VELOCITY_ERROR`, scoring
   each trial.
3. **Choose** — the fastest trial that passed. If the next stiffer trial failed, the winner is
   at the edge of stability, so it backs off one step to keep a margin.
4. **Verify** — re-runs the test on the chosen setting.

Results are **live immediately**, but Dashboard values reset on restart, so copy the printed
lines into `Flywheel.java`. If you stop a run part way, the original constants are restored,
so a half-finished sweep never leaves an untested gain active.

Pass criteria are Dashboard settings: `MAX_OVERSHOOT_FRACTION`, `MAX_NOISE` (speed wander once
settled — the number that matters most for a LUT shooter), and `MAX_STEADY_ERROR`. If the gain
barely changes across the sweep, the result screen says so: the knob isn't what limits that
wheel, and `kA` should be re-checked.

The servo turret has its own automatic tester, the Turret Calibrator (Step 7). Servos run their
own position loop, so there are no turret gains to sweep.

### Step 5 — Flywheel Characterizer (Autonomous, group "test")

**Safety: the wheel spins to full speed. Load nothing, keep clear.**

| Gives you | Goes into |
|---|---|
| `kV` | `Flywheel.kV` |
| `kA` | `Flywheel.kA` |
| `kS` | `Flywheel.kS` |

If `kS` comes out negative, the low power levels never reached steady state. Raise `POWER_LOW`
or `HOLD_SECONDS` and re-run.

After the run it also prints `FF_TABLE.put(...)` lines. Paste those into the static block in
`Flywheel` to use the measured voltage curve instead of the straight-line kV/kS model — see
"Feedforward table" below.

### Step 6 — Flywheel Tuner (Autonomous, group "test")

Step-tests the closed loop and reports settle time, overshoot and steady-state error. Graph
`Flywheel : Target Velocity` against `Flywheel : Filtered Velocity`.

| Symptom | Fix |
|---|---|
| Sits off target and stays there | Feedforward. Re-check `kV` and `kS`. |
| Overshoots | Raise `MAX_VELOCITY_ERROR`, or lower `MAX_EFFORT` |
| Slow to recover | Lower `MAX_VELOCITY_ERROR` until it starts to overshoot, then back off |

### Step 7 — Turret Calibrator (Autonomous, group "test")

The turret is three servos geared together, with an external encoder. Servos run their own
position loop, so there is no LQR to tune; what needs measuring is how servo position maps to
turret angle, and how much backlash the gears have.

Set `Turret.TICKS_PER_TURRET_REV` **first** — every angle depends on it.

1. **During init:** D-pad left/right (bumpers for bigger steps) until the turret points exactly
   straight ahead, then press **A**.
2. **Start:** it probes which way the servos turn the turret, sweeps toward −120°, comes back
   up through the same points (that measures backlash), then does a short sweep toward +30°
   planned from what it has learned so it stops well short of the +50° limit. The encoder is
   checked after every move.

| Gives you | Goes into |
|---|---|
| `ZERO_POSITION` | `Turret.ZERO_POSITION` |
| `POSITION_PER_RADIAN` | `Turret.POSITION_PER_RADIAN` |
| backlash (deg) | Should be under `MAX_TRIM` (4°) so encoder trim can correct it |

It warns if the full −310°…+50° window doesn't fit in the servos' travel, if the points aren't
on a straight line (slipping gears, or wrong `TICKS_PER_TURRET_REV`), or if backlash is too
large for trim.

### Step 8 — Intake Sensor Test (TeleOp, group "test")

Live readout of both entrance sensors and the ball count, with manual intake control.

- **`PRESENT_CM`:** hold a POLLEN at each entrance and read the distance, then read it with the
  entrance empty. Set `PRESENT_CM` between the two, nearer the ball reading. Use POLLEN, not
  NECTAR: it's smaller, so it reads farther away. (Beam-break sensors skip this; check
  `BEAM_ACTIVE_LOW` instead.)
- **Color** (color-capable sensors only): hold each color at an entrance and check it reads
  YELLOW / RED / BLUE. Opponent NECTAR is rejected automatically (G408).
- **The 4-ball limit:** intake with A and watch `BALLS HELD`. At 4 the pivot lifts and the
  motor stops; a 5th that gets in is reversed straight back out.
- **Pollen retrieval:** D-pad up runs the flower mechanism down, intakes, and B brings it back up.

---

## Part 3 — Every variable, and where it comes from

See **[parameters.md](parameters.md)**: every tunable value, what it does, how it gets set, and
every hardware name the code needs. It's kept in one place so it can't drift out of date.

## Remember

Dashboard edits are **lost on restart**. Once a number is final, type it into
`pedroPathing/Constants.java` (drive) or the mechanism's class (`Flywheel.java`, `Turret.java`,
`Intake.java`) and commit it.
