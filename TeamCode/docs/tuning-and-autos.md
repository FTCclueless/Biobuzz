# Tuning and autos: the practical guide

Two questions answered here: **how do I write an auto**, and **what do I tune, with which
tester, and where does the number go**.

Everything tuned lives in ONE file:
`subsystems/drive/DriveConstants.java`. It shows up in FTC Dashboard as **DriveConstants**.

---

## Part 1 — Writing an auto

### Yes, it really is just x/y points

```java
Trajectory toGoal = DriveConstants.pathBuilder()
        .start(12, 60, Math.toRadians(0))   // where the robot is right now
        .end(48, 60, 0)                     // where you want it
        .build();
```

Field coordinates, **inches** and **radians**, heading 0 along +x, counter-clockwise positive.
The builder fits a curve through your points, straightens it, works out the fastest speed the
robot's grip allows, and hands back a `Trajectory`.

**Always start with `DriveConstants.pathBuilder()`.** A bare `new PathBuilder()` silently plans
against generic defaults instead of your measured limits.

### Running it

```java
robot.drivetrain.followTrajectory(toGoal);
robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
```

`ExampleAuto` wraps that in a `follow()` helper. Copy it.

### A realistic path

```java
Trajectory cycle = DriveConstants.pathBuilder()
        .start(12, 60, 0)
        .waypoint(30, 52, 3.0)        // third number = CORRIDOR, see below
        .waypoint(52, 40, 3.0)
        .end(88, 36, 0)               // corridor 0 = pin it exactly
        .velocities(0, 0)             // start and end speed
        .slowZone(88, 36, 10, 18)     // within 10 in of (88,36), cap at 18 in/s
        .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
        .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
        .endHeading(Math.toRadians(-45))
        .marker(20, "raiseLift", () -> lift.setGoal(24))       // 20 in along the path
        .markerBeforeEnd(8, "openClaw", () -> claw.open())     // 8 in before the end
        .build();
```

**Corridor** is the one knob worth understanding: how many inches sideways the optimizer may
move that point to straighten the path.

```
corridor = clearance to nearest obstacle
         - half your robot's diagonal
         - how far you trust your odometry
```

Pin the start, the end, and anything you must line up with (corridor `0`). Open-field points can
take 2-4 inches. More corridor means a straighter, faster path.

**Heading choices** — add several and `build()` keeps the fastest:

| Choice | What it does | Use for |
|---|---|---|
| `TANGENT` | Nose points along the path | Long travel legs, usually fastest |
| `TANGENT_REVERSED` | Nose points backwards | Intake mounted on the back |
| `CONSTANT_START` | Holds the starting heading | Short moves |
| `LINEAR` | Sweeps start heading to end heading | Simple repositioning |
| `FIXED_AT_END` | Tangent, then squares up at the end | Scoring approaches |

**Markers fire by distance along the path, not time.** That is what you actually mean by "drop
the intake 6 inches before the ball". On a slow run a time-based marker fires while the robot is
still feet away.

### Chaining without stopping

Give one path a non-zero end velocity and the next a matching start velocity:

```java
Trajectory legA = DriveConstants.pathBuilder().start(12,60,0).end(48,48,0).velocities(0, 25).build();
Trajectory legB = DriveConstants.pathBuilder().start(48,48,0).end(84,36,0).velocities(25, 0).build();
```

### Rules

- Call `.build()` in `init()`, never in the loop. It runs a spline fit and an optimization.
- Set the starting pose to the real one: `robot.drivetrain.setPoseEstimate(startPose)`.
- For a simple move to a point with no path, use `robot.drivetrain.goToPoint(pose, power)`, which
  uses the point-to-point PIDs, not the trajectory follower.

### Build errors you may hit

| Message | Meaning |
|---|---|
| "turn radius of 0.05 in ... cusp" | Waypoints too unevenly spaced; the spline doubled back. Re-space them. |
| "needs at least a start and an end" | Missing `.start()` or `.end()`. |
| "Duplicate/near-duplicate points" | Two waypoints in nearly the same place. |

---

## Part 2 — What to tune, in order

Run these in order. Each one depends on the one before it.

### Step 0 — Measure by hand

| Variable | How |
|---|---|
| `TRACK_WIDTH` | Left wheel centre to right wheel centre, inches |
| `WHEEL_BASE` | Front wheel centre to back wheel centre, inches |

Note `Globals.TRACK_WIDTH` is a **separate** value used by `Pose2d`. Make them agree.

### Step 1 — Localization Test (TeleOp group "test")

Nothing else works if the robot does not know where it is. Mark the starting spot, drive a lap,
return to the mark. Under about 1 inch and 2 degrees of error is good.

Produces no constants. It is a go/no-go gate.

### Step 2 — Drive Characterizer (Autonomous, group "test")

Needs ~8 ft of clear straight lane and a couple of feet either side. Runs forward x2, strafe x2,
turn x1.

| Gives you | Goes into |
|---|---|
| `V_FORWARD` | `DriveConstants.V_FORWARD` |
| `V_STRAFE` | `DriveConstants.V_STRAFE` |
| `MAX_WHEEL_SPEED` | `DriveConstants.MAX_WHEEL_SPEED` |
| starting `HEADING_GAIN`, `CONTOUR_GAIN`, `LAG_GAIN`, `ACCEL_LEAD` | the matching `DriveConstants` fields |
| `strafeGain` (if > 1.15) | `DriveConstants.STRAFE_GAIN` |

Its `A_FORWARD` / `A_STRAFE` numbers are what the robot *achieved*, not the traction limit.
Step 3 measures the real ones.

### Step 3 — Slip Test (Autonomous, group "test"), run TWICE

It compares the rightRear drive motor's own encoder (wheel speed) against odometry (ground
speed), so that motor's encoder cable must be plugged in.

| Run | Gives you | Goes into |
|---|---|---|
| `STRAFE = false` | `A_FORWARD` | `DriveConstants.A_FORWARD` |
| `STRAFE = true` | `A_STRAFE` | `DriveConstants.A_STRAFE` |

If it says "NEVER SLIPPED", the motors ran out before the tyres did. Use the number anyway; it
is a real ceiling.

If it warns the baseline ratio is above `SLIP_RATIO`, raise `SLIP_RATIO` as it suggests and
re-run. This is common on the strafe axis, where rollers scrub sideways even with full grip.

### Step 4 — Pathing Tuner (Autonomous, group "test")

First with `OPEN_LOOP = true`. In Dashboard, graph these pairs:

- `Profile : planned v` against `Profile : actual v`
- `Profile : planned a` against `Profile : actual a`

Open loop judges the *shape*, not the error. If actual falls well short of planned, your limits
are too optimistic — lower `SAFETY` or re-measure. Large one-sided drift means `TRACK_WIDTH`,
`WHEEL_BASE` or odometry is wrong; fix that before touching gains.

Then set `OPEN_LOOP = false` and follow the "next change" line on telemetry after each run. It
tells you exactly which constant to move and to what. Order of tuning: `HEADING_GAIN`, then
`CONTOUR_GAIN`, then `ACCEL_LEAD`.

| Symptom | Fix |
|---|---|
| Saturated >10% of the run | Lower `SAFETY`. Gains mean nothing until this clears. |
| Weaving, error changes sign often | Lower `CONTOUR_GAIN` |
| Constant offset to one side | Feedforward/geometry, not a gain. Check `STRAFE_GAIN` and the ellipse. |
| Lag only while accelerating | Raise `ACCEL_LEAD` |
| Loose but not weaving | Raise `CONTOUR_GAIN` |

Re-run on `SHAPE` 0 (straight), 1 (L) and 2 (out-and-back).

### Step 5 — Gain Sweeper (Autonomous, group "test"), optional

Sweeps one gain automatically and reports where it went unstable, then backs off by
`MARGIN_FRACTION`. Set `TARGET` to `CONTOUR`, `HEADING` or `ACCEL_LEAD`. It takes the gains it
is not sweeping from `DriveConstants`.

### Steps 6–7 the fast way — Flywheel Auto Tuner

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

The servo turret has its own automatic tester, the Turret Calibrator (Step 8). Servos run their
own position loop, so there are no turret gains to sweep.

### Step 6 — Flywheel Characterizer (Autonomous, group "test")

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

### Step 7 — Flywheel Tuner (Autonomous, group "test")

Step-tests the closed loop and reports settle time, overshoot and steady-state error. Graph
`Flywheel : Target Velocity` against `Flywheel : Filtered Velocity`.

| Symptom | Fix |
|---|---|
| Sits off target and stays there | Feedforward. Re-check `kV` and `kS`. |
| Overshoots | Raise `MAX_VELOCITY_ERROR`, or lower `MAX_EFFORT` |
| Slow to recover | Lower `MAX_VELOCITY_ERROR` until it starts to overshoot, then back off |

### Step 8 — Turret Calibrator (Autonomous, group "test")

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

### Step 9 — Intake Sensor Test (TeleOp, group "test")

Live readout of both entrance sensors and the ball count, with manual intake control.

- **`PRESENT_CM`:** hold a POLLEN at each entrance and read the distance, then read it with the
  entrance empty. Set `PRESENT_CM` between the two, nearer the ball reading. Use POLLEN, not
  NECTAR: it's smaller, so it reads farther away. (Beam-break sensors skip this; check
  `BEAM_ACTIVE_LOW` instead.)
- **Color** (color-capable sensors only): hold each color at an entrance and check it reads
  YELLOW / RED / BLUE. Opponent NECTAR is rejected automatically (G408).
- **The 4-ball limit:** intake with A and watch `BALLS HELD`. At 4 both pivots lift and the
  motors stop; a 5th that gets in is reversed straight back out.
- **Pollen retrieval:** D-pad up runs the flower mechanism down, intakes, and B brings it back up.

---

## Part 3 — Every variable, and where it comes from

See **[parameters.md](parameters.md)**: every tunable value, what it does, how it gets set, and
every hardware name the code needs. It's kept in one place so it can't drift out of date.

## Remember

Dashboard edits are **lost on restart**. Once a number is final, type it into the default in
`DriveConstants.java` (or `Flywheel.java`) and commit it.
