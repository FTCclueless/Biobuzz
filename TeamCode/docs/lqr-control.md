# LQR mechanism control

`control/` holds a small LQR controller for motors that need to hold a speed or a
position: `Matrix`, `LQR`, `MechanismModel`, `MotionProfile`, `StatePredictor` and
`LQRController`. The flywheel uses it (see `subsystems/shooter/Flywheel.java`). Driving
and pathing are handled by Pedro Pathing, not by this package.

## Usage

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

## Measuring the mechanism constants (kV, kA, kS, kG)

- **kS** — raise voltage from zero until the mechanism *just* begins to move. That voltage.
- **kG** *(vertical only)* — the voltage at which the slide neither rises nor falls. Do
  this loaded, the way it will run in a match.
- **kV** — hold a few steady voltages, record steady-state velocity, and fit
  `V = kS + kG + kV·v`. The slope is kV.
- **kA** — step the voltage and time the approach to steady state. The system is a
  first-order lag with time constant `kA/kV`; the time to reach 63% of final velocity is
  that time constant, so `kA = kV × tau`. Sanity check: tau should land between about
  0.05 s and 0.5 s.

For the flywheel, the Flywheel Characterizer measures these for you.

## Checklist

- [ ] **kG is not optional on a vertical slide.** LQR has no integrator, so a constant
      gravity disturbance produces a constant error of exactly `kG / positionGain` — there
      is nothing in the loop to drive it to zero. Raising the gain hides it until the
      battery sags or the load changes. Feedforward is the fix.
- [ ] **Voltage-compensate.** Without it every gain silently scales with battery charge.
- [ ] **Pass your real loop period.** Gains computed for 20 ms and applied at 40 ms are
      twice as aggressive as intended.
- [ ] **Compensate latency.** 20–40 ms of loop delay is normal, and feedback against a
      stale state shows up as overshoot no amount of gain tuning removes.
- [ ] **Never position-control a flywheel.** Its position is not a thing you want to
      regulate; `setGoal` throws on a `VELOCITY` mechanism rather than let you.
- [ ] **Profile position moves.** An unprofiled step command saturates the motor and makes
      the feedforward meaningless for the whole saturated stretch.
