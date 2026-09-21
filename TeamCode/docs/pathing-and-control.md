# FTC Path Planner + LQR Control

A self-contained path planning / following library and an LQR mechanism controller for an
FTC mecanum robot. **Pure Java 8, zero external dependencies** — no QP solver, no native
code, no JUnit. It lives alongside the rest of the robot code in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```
org.firstinspires.ftc.teamcode
├── pathing
│   ├── geometry    BezierSegment, Path (arc-length), SplineFitter, Waypoint, PathBuilder
│   ├── opt         BoxQP, MinCurvatureOptimizer
│   ├── profile     FrictionEllipse, HeadingPlan, VelocityProfile, RegionConstraint, Trajectory
│   └── follow      MecanumKinematics, GuidingVectorField, SlipDetector, PathFollower, IterativeLearner
└── control
    └── Matrix, LQR, MechanismModel, MotionProfile, LQRController
```

The library has no geometry types of its own — it uses the robot's `utils.Vector2` and
`utils.Pose2d`, with numerics in `utils.MathUtil`, `utils.Utils` (`headingClip`,
`minMaxClip`) and `utils.RateEstimator`. `pathing` holds path planning and nothing else.

Treat a `Vector2` handed to or returned by this library as a **value**. `Vector2` is
mutable, but every operation the planner uses (`plus`, `minus`, `times`, `unit`,
`rotated`, …) returns a new instance and the library caches the results. Calling a
mutating method (`add`, `mul`, `rotate`, `norm`) on a vector you got back from a path or
trajectory will corrupt it in place.

One name to keep straight: `pathing.geometry.Path` is the planner's arc-length path, not
the robot's existing `subsystems.drive.Path`.

## Quick start

### Build and run a path

```java
Trajectory traj = new PathBuilder()
        .start(12, 60, Math.toRadians(0))
        .waypoint(30, 52, 3.0)          // 3 inches of lateral freedom for the optimizer
        .waypoint(52, 40, 3.0)
        .end(88, 36, 0)                 // pinned — a scoring position
        .velocities(0, 0)
        .slowZone(88, 36, 10, 18)       // arrive under control
        .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
        .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
        .endHeading(Math.toRadians(-45))
        .marker(20, "raiseLift", () -> lift.setGoal(24, 40, 120))
        .markerBeforeEnd(8, "openClaw", () -> claw.open())
        .build();

MecanumKinematics kin = new MecanumKinematics(14, 13, 66);   // track, base, wheel in/s
PathFollower follower = new PathFollower(traj, kin);

// in the OpMode loop
PathFollower.Command cmd = follower.update(odometry.getPose(), loopTime);
fl.setPower(cmd.powers[0]);
fr.setPower(cmd.powers[1]);
bl.setPower(cmd.powers[2]);
br.setPower(cmd.powers[3]);
if (follower.isFinished()) { /* next path */ }
```

`build()` fits a C2 cubic spline through the waypoints, runs the min-curvature optimizer
inside your corridors, profiles **every** heading candidate against the friction ellipse,
and returns the fastest.

### LQR for slides and flywheels

```java
// Vertical lift — kG is what stops it sagging
LQRController lift = new LQRController(
        MechanismModel.verticalSlide(0.130, 0.0110, 0.090, 1.35), 0.020)
        .tolerances(new double[]{0.30, 6.0}, 10.0)   // 0.30 in, 6 in/s, 10 V
        .latencyCompensation(0.030);

lift.setGoal(24.0, 40.0, 120.0);                     // target, max vel, max accel
liftMotor.setPower(lift.calculate(inches, inchesPerSec, batteryVolts));

// Flywheel — velocity only, no profile, maximum recovery rate
LQRController shooter = new LQRController(
        MechanismModel.flywheel(0.0210, 0.00340, 0.120), 0.020)
        .tolerances(new double[]{10.0}, 10.0);

shooter.setVelocityGoal(340.0);                      // rad/s
shooterMotor.setPower(shooter.calculateVelocity(radPerSec, batteryVolts));
if (shooter.atVelocity(radPerSec, 8)) { /* fire */ }
```

Horizontal slides are the same as vertical with `kG = 0`:
`MechanismModel.horizontalSlide(kV, kA, kS)`.

### Building

```bash
./gradlew :TeamCode:compileDebugJavaWithJavac
```

The library's own test suite, simulator, and demo were not merged into this repository —
they live with the standalone library at `~/FTC`. Run them there against a copy of these
sources if you change the planner's internals.

---

## Measuring the friction ellipse

Every number in the planner comes from here. **Do not** copy the defaults in
`FrictionEllipse.typicalFtc()` — they are a starting point, not a spec.

### The slip-push test

The key rule: **never differentiate odometry to get acceleration.** Encoder quantization
and loop jitter land directly on top of the signal you want, and you end up tuning against
noise. Instead, *compare two independent velocity measurements* and find where they
diverge — that divergence is the traction limit, measured directly.

1. Put the robot on the real competition surface (foam tiles behave nothing like your
   shop floor), with a typical match load and a **freshly charged battery**.
2. Wire up a `SlipDetector` with both velocity sources:
   ```java
   detector.update(
       kinematicsFromDriveEncoders,   // what the wheels think
       velocityFromDeadWheels,        // what the robot is actually doing
       loopTime);
   ```
3. Ramp the drive power up smoothly — roughly 0 to 1.0 over about two seconds. Ramping
   matters: slam it to full and you slip instantly and measure nothing.
4. Watch `detector.isSlipping()`. The moment it trips, the wheels have broken traction.
5. Read `detector.peakTractionAccel()` — the largest acceleration observed *while still
   gripping*. That is your `aForward`.
6. Repeat driving **sideways** for `aStrafe`. Expect roughly 65–80% of forward; mecanum
   rollers give up sooner than the wheels do.
7. For the speed axes, drive a long straight at full power until velocity plateaus.
   That plateau is `vForward`; repeat strafing for `vStrafe`.
8. `stallRatio` = (stall torque available at the wheel) / (torque traction can accept).
   For a typical FTC drivetrain this is **3–4**. If you compute below ~2, re-check your
   gear ratio; below 1 would mean the motors cannot break traction at all, which is not
   how these robots behave.

Then plan at ~85% of what you measured (`.safetyFactor(0.85)`), so the follower has
authority left over for feedback. See pitfall #7.

### Measuring the mechanism constants (kV, kA, kS, kG)

- **kS** — raise voltage from zero until the mechanism *just* begins to move. That voltage.
- **kG** *(vertical only)* — the voltage at which the slide neither rises nor falls. Do
  this loaded, the way it will run in a match.
- **kV** — hold a few steady voltages, record steady-state velocity, and fit
  `V = kS + kG + kV·v`. The slope is kV.
- **kA** — step the voltage and time the approach to steady state. The system is a
  first-order lag with time constant `kA/kV`; the time to reach 63% of final velocity is
  that time constant, so `kA = kV × tau`. Sanity check: tau should land between about
  0.05 s and 0.5 s.

---

## Tuning order

Do these **in order**. Each step assumes the previous one is right, and tuning feedback
gains on top of a broken feedforward is how people end up with gains that work on one
battery and one field.

1. **Odometry.** Push the robot a known 96 inches and check it reads 96. Spin it 10 times
   and check the heading. Nothing downstream can be better than this.
2. **Friction ellipse.** Slip-push, as above. Plan at 85%.
3. **Feedforward, open loop.** Set every feedback gain to zero and run a path. The robot
   should roughly follow it. If it does not, the ellipse is wrong — fix that, do not paper
   over it with feedback.
4. **Heading gain.** Tune `headingGain` alone on a straight path until heading is crisp.
5. **Contour gain.** Raise `PathFollower.contourGain` until perpendicular error is small
   and the robot does not weave. This is the gain that matters.
6. **Lag gain.** Leave `PathFollower.lagGain` low — 3–4× below the contour gain. It exists
   to keep the reference from running away, not to catch up.

   > Corrective stiffness comes from two places: the **GVF's tilt** toward the path (a
   > geometric, velocity-level effect that supplies most of the pull at speed) and the
   > **follower's P-term** on top. Both expose gains named `contourGain`/`lagGain` because
   > they act on the same two error components. Tune the `PathFollower` ones; the
   > `gvf()` defaults are usually fine. ILC learns from the sum of the two.
7. **Accel lead.** 30–50 ms. Raise until the robot stops trailing on acceleration.
8. **ILC, last.** Only once everything above is stable. `gamma = 0.5`, run the path 8
   times, save the table.

For the LQR side, tuning is just `tolerances(...)`: state the error you actually care
about in inches and in/s, and the effort you are willing to spend in volts. Tighter
tolerance → higher gain, quadratically. You should not be hand-editing gains.

---

## The pitfalls, as a checklist

These are the whole point of the library. Each has a test that fails if it regresses.

- [ ] **1. Arc-length reparameterization.** Bezier `t` is *not* distance. A uniform `dt`
      walks the curve at wildly varying speed — on this library's own test path, uniform-`t`
      sampling ripples **1006%** while uniform-`s` ripples **0.5%**. Everything downstream
      (profile lookup, GVF, markers) assumes inches. Skip this and you get speed ripple
      that is miserable to debug.
      → *Test: uniform-s spacing ripple < 1%.*

- [ ] **2. The min-curvature QP diverges without a trust region.** The QP linearizes
      curvature by freezing the `(x'²+y'²)^{3/2}` denominator. That is only valid locally,
      and at FTC radii (inches, not metres) a full QP step can land somewhere the *true*
      curvature is far worse — 20× worse, observed. After every QP step, recompute the true
      nonlinear cost and accept only on real improvement; otherwise reject and halve the
      trust region. The linear model does not get a vote.
      → *Test: strict cost decrease on a wiggly path; steps actually get rejected.*

- [ ] **3. The s-indexed stall.** `v(0) = 0`, the robot is on the path so there is no error,
      so it commands zero forever and never moves. Verified in the test suite: with the
      floor removed the robot moves **exactly 0.0000 inches**. Seed a small velocity floor
      over the first ~2 inches. On a field this presents as "the robot just doesn't go
      sometimes", which is the worst thing to debug at a competition.
      → *Test: robot escapes the equilibrium; stalls dead without the floor.*

- [ ] **4. C2 cubic splines inside the optimizer, not quintic.** The point → second-derivative
      map for a C2 cubic is a fixed linear operator, which is exactly what keeps the problem
      a QP. Quintic is for hand-off joints where you must pin curvature.

- [ ] **5. Acceleration is two-regime, and it is not one constant.** Available accel is
      `min(traction, stallRatio × (1 − v/vTop))` — flat at low speed, falling only near the
      top. An FTC drivetrain has ~3–4× more stall torque than traction can use, so a derate
      falling linearly from `v = 0` is **~45% too pessimistic** in the 15–40 in/s band where
      short FTC moves live. **Only derate the accelerating side** — braking is traction
      limited, not back-EMF limited, and derating it plans a stop the robot beats, landing
      short of the path end where an s-indexed robot parks forever.
      → *Test: accel flat at 5 and 30 in/s, falling at 58; braking full at all speeds.*

- [ ] **6. Contour error ≠ lag error.** Perpendicular error costs you; being a couple of
      inches behind *along* the path is nearly free. In this library's own test, a 4-inch
      bump across the path produces 4.08 in of contour error, while the same 4-inch bump
      *along* it produces 0.25 in. Weight them separately (high contour, low lag) and apply
      corrections in the **body frame** with separate forward/strafe gains. One isotropic XY
      gain spends authority fighting error that does not matter.

- [ ] **7. Reserve traction, and yield tangential first.** Plan at ~85% of the ellipse so
      feedback has authority left. On saturation, give up along-path speed *before* giving
      up perpendicular tracking — arriving late is cheap, leaving the path corrupts odometry
      and kills the run. Normalize the wheels **proportionally**: clipping them individually
      changes the ratio between them, which changes the direction the robot travels.

### LQR checklist

- [ ] **kG is not optional on a vertical slide.** LQR has no integrator, so a constant
      gravity disturbance produces a constant error of exactly `kG / positionGain` — there
      is nothing in the loop to drive it to zero. Raising the gain hides it until the
      battery sags or the load changes. Feedforward is the fix.
- [ ] **Voltage-compensate.** Without it every gain silently scales with battery charge.
- [ ] **Pass your real loop period.** Gains computed for 20 ms and applied at 40 ms are
      twice as aggressive as intended.
- [ ] **Compensate latency.** 20–40 ms of loop delay is normal, and feedback against a
      stale state shows up as overshoot no amount of gain tuning removes (measured here:
      1.73 in of overshoot uncompensated, ~0 compensated).
- [ ] **Never position-control a flywheel.** Its position is not a thing you want to
      regulate; `setGoal` throws on a `VELOCITY` mechanism rather than let you.
- [ ] **Profile position moves.** An unprofiled step command saturates the motor and makes
      the feedforward meaningless for the whole saturated stretch.

---

## Testing philosophy

The simulator is **deliberately worse than the planner's model** — 12% weaker acceleration,
30 ms actuation lag, a dragging wheel, and a different stall ratio (2.8 vs the planner's
3.5). The mechanism sim is likewise mismatched: 15% heavier, 8% more drag, 40% more
stiction. A simulator built from the controller's own equations will happily validate a
controller that is wrong; it proves only that the code is self-consistent.

The robustness suite runs pose noise, odometry drift, a mid-path perpendicular bump, a
robot 30% weaker than planned, a low battery, a 33 ms loop, and all of it at once. The
required outcome is always the same: **the robot completes**. Late and on-path is the
correct failure mode; divergence is a bug.

On top of the hand-written cases, `StressTests` fuzzes the whole pipeline — 300 random
paths, 200 random profiles with deliberately infeasible terminal velocities, 120 random
optimizations, and 60 random closed-loop runs. Nothing may produce NaN, throw, or violate a
stated invariant. The hand-written tests check the cases someone thought of; this checks
the ones nobody did. It is how the cusp problem below was found.

```
640 passed, 0 failed
```

## Defect classes

The suite was used as a review target: two independent correctness passes plus the fuzzer
found **eleven real defects**, every one silent. Each got a regression test for the
instance — and, more usefully, a structural fix and a property test for the *class*. A
per-instance test catches the bug you already found; a property test catches the next one.

`InvariantTests` holds one section per class below. All eleven fixes were **mutation-tested**:
each defect was deliberately reintroduced and the suite confirmed to fail.

| # | Defect (instance) | Inherent problem | Structural fix |
|---|---|---|---|
| 1 | Path shorter than the grid resolution → 2-node profile → a 0.4 in move reported as **400 s** | Grid sized by a bare ratio degenerates on small domains — and the divide-by-zero *guard* converted that into a plausible wrong number instead of an error | `MathUtil.gridNodes(length, res, minNodes)` enforces a floor everywhere a grid is built. Property test: duration is **continuous in path length**, no cliff |
| 2 | `Linear` heading plan fabricated an angular-accel cap at both path ends, *tighter* as the grid refined | Finite difference taken across a function's domain edge, where the function reports 0 outside — plus dividing by the nominal rather than actual span | `MathUtil.derivative`/`angleDerivative` clamp samples and divide by the true span; heading plans are **constant-extended, never zeroed**. Property test: refining the step must not change the answer |
| 3 | Reference station derived from the robot's own projection → `lagError ≡ vRef·dt` | A "measurement" computed from the thing it is measured against is algebraically constant and can never respond | Independent reference station, clamped to a bounded lead. Property test: **signal liveness** — each diagnostic must move when its named condition is applied, and *not* when an unrelated one is |
| 4 | A cycle path reported "finished" before the robot moved | Termination predicate satisfiable in the initial state | Completion requires goal **and** progress. Property test: across 60+ shapes including closed loops, no follower is finished after one update |
| 5 | A marker at the path end silently never fired | Two different notions of "the end" — the loop stops at `endpoint − tolerance` while events index the full domain | Flush-on-terminate. Property test: **no marker is ever dropped** on a completed run, with markers stacked in the last inch |
| 6 | `peakTractionAccel()` read 12× high after a slip — the number the slip-push procedure tells you to trust | Differencing across a gap in sampling: the previous sample isn't one `dt` old | `RateEstimator` tracks baseline adjacency and refuses to produce a rate across a gap. Prefer *comparing two measurements* over differentiating one |
| 7 | `safetyFactor()` silently replaced your measured ellipse with the sample one, order-dependently | Order-dependent fluent setters — one setter consuming another's input | Builders store *intent* and resolve at `build()`. Property test: **six setter orderings produce byte-identical trajectories**, and repeats are last-wins |
| 8 | `markerBeforeEnd(0, …)` fired at the **start** (`-0.0` is not `< 0`) | In-band sentinel — using a value's sign to carry a flag, when the range includes the boundary | Explicit flag, no sentinel. Property test: offsets at `0.0`, `-0.0`, `1e-12` and past both ends all resolve correctly |
| 9 | Zero-length path → `ds = 0` → NaN acceleration into the follower's feedforward | NaN raises nothing; it flows through arithmetic silently until it reaches the motors | Validate degenerate inputs, plus **finite-guards on both the follower's input and output** — a non-finite pose returns a full stop and sets `fault()` rather than propagating. Zeroing, not throwing: an exception kills the OpMode mid-match |
| 10 | `lowerBound` returned `-1` on a 1-element table, contradicting its own contract | Contract broken at the smallest valid input; a public helper with an undocumented precondition | Guarded, and property-tested at sizes **1–4 against six query positions** each, alongside `interp`, `interpAngle` and `sampleUniform` |
| 11 | Latency model rebuilt every loop — a matrix exponential and ~41 allocations at 50 Hz, behind a comment claiming it was precomputed | Configuration-time work leaking into the hot loop. **Comments don't fail builds** | `Matrix.allocations` counter; the test asserts the control loop allocates **exactly zero** matrices over 2000 iterations, for every mechanism type and both latency settings |

Two themes run through most of these, and both are worth internalizing beyond this codebase:

- **A guard that turns a degenerate state into a plausible number is worse than no guard.**
  Defects 1 and 9 both had one. `max(v, 1e-3)` and a silent NaN are the same mistake:
  they convert "this is broken" into "this is fine, here's an answer."
- **The tell for a boundary artifact is that refining makes it worse.** Defect 2 got
  *tighter* as the grid got finer. Any quantity that diverges under refinement is an
  artifact of how you're measuring, not a property of what you're measuring.

### Cusps

Fitting a spline through unevenly spaced waypoints can produce a **cusp** — a point where
the curve doubles back on itself, `|dP/dt|` collapses toward zero, and curvature explodes.
The fuzzer found one at 21.9 1/in: a corner of radius 0.046 inches. It is geometrically
valid and physically absurd, and because it is far narrower than the profile grid, a naive
point-sampled speed cap walks straight past it.

Two defences, both in the library:

- `VelocityProfile` samples curvature as the **maximum over each grid interval**, not the
  value at the node, so a spike narrower than one cell cannot alias away.
- `PathBuilder.build()` rejects any path whose turn radius drops below `minTurnRadius`
  (default 1 inch — about 4× tighter than any FTC robot can drive, so it never fires on a
  real path). You get a clear error during `init()` instead of a robot that tries to drive
  it. `Path.hasCusp()` and `Path.peakCurvature()` are public if you want to check yourself.

If it fires, one of your segments is much shorter than its neighbours. Re-space the
waypoints.

### Performance

Measured on a laptop; a Control Hub is roughly 10–30× slower for scalar code, so scale
accordingly — the margins are still very large.

| Operation | Cost | Notes |
|---|---|---|
| `PathBuilder.build()` | ~31 ms | **offline** — call in `init()`, never in the loop |
| `PathFollower.update()` | **0.8 µs** | against a 20,000 µs loop budget |
| `Path.project()` | **0.53 µs** | dominates the follower |
| `Path.sample()` | 0.08 µs | allocation-free station query |
| `LQRController.calculate()` | 0.02 µs | allocation-free, including latency compensation |

**Allocation per control loop: 248 bytes** (nose-forward), down from 13,224. That matters
more than the microseconds: on Android, per-loop allocation becomes GC pauses, GC pauses
become loop-period jitter, and jitter detunes a controller whose gains were computed for a
fixed `dt`.

Where it went:

- `BezierSegment` evaluates degrees ≤ 4 by direct Bernstein form, and exposes
  `speed(t)` / `pointInto(…)` / `curvature(t, scratch)` that write components instead of
  returning a `Vec2`. Arc-length quadrature calls `speed()` ten times per inversion — going
  through `deriv().norm()` built and discarded a vector on every one.
- `Path.sample(s, Sample)` fills a caller-owned struct; `project(qx, qy, …, Sample)` runs
  its coarse basin scan on a table-only inversion (no Newton, no quadrature) and refines
  only the winner. Same answer, a fraction of the work.
- `PathFollower` holds one `Sample` and one `GVF.Field` for the life of the run and does
  the frame rotations and correction sums in scalars.
- `HeadingPlan.Tangent` reads its heading and rate straight off the sample — nose-forward
  *is* the path's own tangent and curvature, so re-deriving them was a second arc-length
  inversion per loop.
- `LQRController` precomputes its latency-prediction model at configuration time rather
  than redoing a matrix exponential every loop.

Accuracy is unchanged — **bit-identical** on a 200-path randomized comparison of
round-trip error, tangent unit-norm, arc-length ripple, and projection against a
4000-sample brute-force search. Both the allocation budget and the zero-matrix rule are
enforced by tests, so this cannot silently regress.

`PathFollower.update()` is deliberately *not* claimed to be zero-allocation: it still
builds a `Command` and its power array each loop, because handing back a reused mutable
object would be a correctness trap for anyone who stores it. 248 bytes at 50 Hz is 12 KB/s,
which the young generation absorbs without noticing.

Several of the pitfalls above were originally caught by a test failing on a path shape an
earlier test did not exercise. After any change, run the whole suite.
