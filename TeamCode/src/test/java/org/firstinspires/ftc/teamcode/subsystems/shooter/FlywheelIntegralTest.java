package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.control.LQRController;
import org.firstinspires.ftc.teamcode.control.MechanismModel;
import org.firstinspires.ftc.teamcode.control.StatePredictor;
import org.junit.Test;

public class FlywheelIntegralTest {
    static final double kV = 0.019436, kA = 0.002090, kS = 1.5667, DT = 0.02;

    /** Steady speed after 3 s at the target, with the feedforward over-estimating by biasVolts. */
    static double settle(double kI, double biasVolts) {
        double old = Flywheel.kI;
        Flywheel.kI = kI;
        try {
            LQRController c = new LQRController(MechanismModel.flywheel(kV, kA, kS), DT)
                    .tolerances(new double[]{14.7}, 8.0).outputLimits(0, 12).voltageCompensation(true);
            double a = Math.exp(-kV / kA * DT), b = (1 - a) / kV;
            double v = 370, target = 470, integral = 0;
            c.setVelocityGoal(target);
            for (int i = 0; i < 3 / DT; i++) {
                double power = c.calculateVelocity(v, 12.0, kS + kV * target + biasVolts + integral);
                integral = Flywheel.integrate(integral, target - v, DT, c.isSaturated());
                v = a * v + b * (power * 12.0 - kS);
            }
            return v;
        } finally {
            Flywheel.kI = old;
        }
    }

    @Test
    public void withoutIntegralAFeedforwardErrorLeavesAnOffset() {
        // The bug seen on the robot: ~1 V too much feedforward settles ~10 in/s high.
        assertTrue(settle(0, 1.1) > 478);
    }

    @Test
    public void integralRemovesTheOffset() {
        assertEquals(470, settle(0.2, 1.1), 1.0);
        assertEquals(470, settle(0.2, -1.1), 1.0);
    }

    @Test
    public void integralHoldsFarBelowTargetButNotFarAbove() {
        // Below: spin-up and shot dips, must not wind up. Above with the output unclipped: the
        // feedforward is too high, and freezing here left the robot ~100 in/s fast.
        assertEquals(0.0, Flywheel.integrate(0, 100, 0.02, false), 0);
        assertTrue(Flywheel.integrate(0, -100, 0.02, false) < 0);
        assertEquals(0.0, Flywheel.integrate(0, -100, 0.02, true), 0);
    }

    @Test
    public void predictorHoldsSteadySpeedWhenVoltageJustCoversFriction() {
        // At steady state the wheel neither speeds up nor slows down, so the prediction must match.
        StatePredictor p = StatePredictor.create(MechanismModel.flywheel(kV, kA, kS), 0.04);
        p.update(0, 470, kS + kV * 470);
        assertEquals(470, p.velocity(), 0.01);
    }

    @Test
    public void currentLimitHoldsAtStandstill() {
        // At 0 speed the motor draws the friction current plus MAX_MOTOR_AMPS of accelerating current.
        double frictionAmps = Flywheel.FF_TABLE.volts(0) / Flywheel.MOTOR_RESISTANCE;
        assertEquals(Flywheel.MAX_MOTOR_AMPS + frictionAmps,
                Flywheel.motorAmps(Flywheel.maxVolts(0), 0), 1e-9);
    }

    @Test
    public void currentLimitNeverBlocksHoldingSpeedEvenWithHighFriction() {
        // A re-characterized wheel with stiff belts: 6 V just to turn. The old back-EMF cap
        // (back-EMF + 5.2 V) sat below the hold volts here and left the wheel slow.
        java.util.Map<Double, Double> saved = new java.util.TreeMap<>();
        for (double v = 0; v <= 600; v += 1) saved.put(v, Flywheel.FF_TABLE.volts(v));
        double[][] old = {{166.2, 4.736}, {259.7, 6.709}, {362.8, 8.629}, {463.6, 10.509}, {554.2, 12.361}};
        try {
            Flywheel.FF_TABLE.clear();
            Flywheel.FF_TABLE.put(100, 6.0 + 0.0196 * 100).put(500, 6.0 + 0.0196 * 500);
            for (double v = 0; v <= 300; v += 25) {
                assertTrue("at " + v, Flywheel.maxVolts(v) >= Math.min(12.0, Flywheel.FF_TABLE.volts(v)) + 1.0
                        || Flywheel.maxVolts(v) == 12.0);
            }
        } finally {
            Flywheel.FF_TABLE.clear();
            for (double[] p : old) Flywheel.FF_TABLE.put(p[0], p[1]);
        }
    }

    @Test
    public void currentLimitNeverBlocksHoldingSpeed() {
        // The volts that hold a speed must fit under the cap at that speed, or the wheel would
        // settle below its target (the old spin-up-only limit existed to avoid exactly that).
        for (double v = 0; v <= 550; v += 25) {
            assertTrue("at " + v, Flywheel.maxVolts(v) >= Math.min(12.0, Flywheel.FF_TABLE.volts(v)) + 1.0
                    || Flywheel.maxVolts(v) == 12.0);
        }
    }
}
