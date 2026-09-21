# Getting started: building your first path

## 1. Where the library lives

Everything sits under the normal package root, next to the rest of the robot code:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

| Folder | Files | What it is |
|---|---|---|
| `pathing/geometry/` | 6 | Beziers, arc-length `Path`, spline fitter, `PathBuilder` |
| `pathing/opt/` | 2 | `BoxQP`, `MinCurvatureOptimizer` |
| `pathing/profile/` | 5 | Friction ellipse, heading plans, velocity profile, `Trajectory` |
| `pathing/follow/` | 5 | Kinematics, GVF, slip detector, `PathFollower`, ILC |
| `control/` | 5 | `Matrix`, `LQR`, `MechanismModel`, `MotionProfile`, `LQRController` |

Geometry and numerics come from the robot's own `utils` package — `Vector2`, `Pose2d`,
`MathUtil`, `Utils.headingClip` / `Utils.minMaxClip`, `RateEstimator`. The planner defines
no vector or pose type of its own, so a pose read off the localizer feeds
`PathFollower.update` directly with no conversion.

`control/` references nothing else, and `pathing/` never references `control/`, so the two
halves stay independent.

`Vector2` is mutable, but the planner treats it as a value: it stores and reuses the
vectors it hands back. Use the copying operations (`plus`, `minus`, `times`, `unit`,
`rotated`) on anything that came from a `Path` or `Trajectory`; the in-place ones (`add`,
`mul`, `rotate`, `norm`) will corrupt the planner's own state.

One name to keep straight: `pathing.geometry.Path` is **not** `subsystems.drive.Path`.
Both exist; import the one the surrounding code already uses.

The library's test suite, simulator, and `Demo` were deliberately left out of this
repository — they never run on the robot and would only bloat the APK. They remain with
the standalone library at `~/FTC` if you need to re-verify a change to the planner.

---

## 2. Creating a path

### The mental model

You describe **where the robot must go**, not how it gets there. The builder fits a smooth
curve through your waypoints, straightens it within the room you allow, works out how fast
it can be driven given your robot's grip, and hands back a `Trajectory` that the follower
executes.

Everything is in **inches** and **radians**, in field coordinates. Headings are CCW-positive
with 0 along +x.

### Minimal path

```java
Trajectory traj = new PathBuilder()
        .start(12, 60, Math.toRadians(0))   // where the robot starts, and its heading
        .end(60, 60, 0)                     // where it ends; 0 = no lateral freedom
        .build();
```

That is a valid straight-line trajectory. `build()` is doing real work — spline fit,
optimization, friction-ellipse profiling — so call it in `init()`, never in the loop.

### A realistic path

```java
Trajectory traj = new PathBuilder()
        // Start pinned at the robot's actual starting pose.
        .start(12, 60, Math.toRadians(0))

        // Intermediate waypoints. The third number is the CORRIDOR: how many inches
        // sideways the optimizer may move this point to straighten the path.
        //   0   = pin it exactly (an alignment point, a gate)
        //   2-4 = normal open-field travel
        .waypoint(30, 52, 3.0)
        .waypoint(52, 40, 3.0)

        // The end is pinned — this is a scoring position.
        .end(88, 36, 0)

        // Terminal speeds. Leave at 0,0 to stop; set nonzero to chain into the next path
        // without stopping.
        .velocities(0, 0)

        // Plan at 85% of measured grip, leaving authority for the follower. See PITFALL #7.
        .safetyFactor(0.85)

        // Slow down near the scoring position: centre, radius, max in/s.
        .slowZone(88, 36, 10, 18)

        // Let the builder try several heading strategies and keep the fastest.
        .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
        .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
        .endHeading(Math.toRadians(-45))

        // Actions, indexed by DISTANCE along the path, not time.
        .marker(20, "raiseLift", () -> lift.setGoal(24, 40, 120))
        .markerBeforeEnd(8, "openClaw", () -> claw.open())

        .build();
```

### Choosing the corridor

The corridor is the single most useful knob. It is how much lateral freedom the optimizer
has at that waypoint, in inches:

```
corridor = (clearance to the nearest obstacle)
         - (half your robot's diagonal)
         - (how far you trust your odometry)
```

Give it room and the path gets straighter and faster; pin it at 0 and the robot goes exactly
where you said. Pin the start, the end, and anything you must line up with. Everything else
can usually take 2–4 inches.

### Heading strategies

| Choice | Behaviour | Use for |
|---|---|---|
| `TANGENT` | Nose along the path | Long travel legs — usually fastest |
| `TANGENT_REVERSED` | Nose backwards along the path | Intake on the back |
| `CONSTANT_START` | Hold the starting heading | Short moves where turning is not worth it |
| `LINEAR` | Sweep start heading → end heading | Simple repositioning |
| `FIXED_AT_END` | Tangent, then square up at the end | Scoring approaches |

Add several with `.headingCandidate(...)` and `build()` profiles each and returns the
fastest. To see the trade rather than just take it:

```java
System.out.print(builder.compareHeadings());
//   TANGENT            2.445 s
//   FIXED_AT_END       2.563 s
//   ...
```

### Markers

Markers fire by **arc length**, not time, which is what you actually mean by "drop the
intake 6 inches before the sample". If the robot runs slow — dead battery, heavy game
element — a time-indexed marker fires while it is still feet away; an s-indexed one fires
where you asked.

```java
.marker(20, "name", runnable)            // 20 inches from the start
.markerBeforeEnd(8, "name", runnable)    // 8 inches before the end
.markerBeforeEnd(0, "name", runnable)    // exactly at the end
```

### Running it

```java
// init()
Trajectory traj = new PathBuilder()...build();
MecanumKinematics kin = new MecanumKinematics(TRACK_WIDTH, WHEEL_BASE, MAX_WHEEL_SPEED);
PathFollower follower = new PathFollower(traj, kin);

// after start()
follower.reset();
while (opModeIsActive() && !follower.isFinished()) {
    double dt = /* measured loop period, seconds */;
    PathFollower.Command cmd = follower.update(odometry.getPose(), dt);
    fl.setPower(cmd.powers[0]);
    fr.setPower(cmd.powers[1]);
    bl.setPower(cmd.powers[2]);
    br.setPower(cmd.powers[3]);

    telemetry.addData("s", "%.1f / %.1f", cmd.s, traj.length());
    telemetry.addData("contour err", "%.2f in", cmd.contourError);
    telemetry.addData("fault", follower.fault());   // null unless something went wrong
    telemetry.update();
}
```

Then stop the motors. Put `follower.fault()` on telemetry — if the follower ever gets a
non-finite pose it stops the robot rather than propagating NaN, and that string is how you
find out.

### Chaining paths

Give the first path a nonzero end velocity and the second a matching start velocity, and
the robot flows through the junction instead of stopping:

```java
Trajectory legA = new PathBuilder().start(12,60,0).end(48,48,0).velocities(0, 25).build();
Trajectory legB = new PathBuilder().start(48,48,0).end(84,36,0).velocities(25, 0).build();
```

### Errors you may hit at build time

**"Path has a turn radius of 0.05 in … the spline almost certainly formed a cusp."**
Your waypoints are spaced such that the spline doubled back on itself — usually one segment
much shorter than its neighbours. Move or re-space them. `minTurnRadius(0)` bypasses the
check if you are certain.

**"A path needs at least a start and an end."** You called `build()` without both.

**"Duplicate/near-duplicate points at index N."** Two waypoints in nearly the same place.

### Before you trust any of the timing numbers

The defaults in `FrictionEllipse.typicalFtc()` are a **starting point, not your robot**.
Until you run the slip-push procedure in the README and put your own numbers in, the
planner is optimizing against a robot you do not have. Everything will still work — it will
just be planned for the wrong grip, which means either leaving time on the table or sliding
in the corners.
