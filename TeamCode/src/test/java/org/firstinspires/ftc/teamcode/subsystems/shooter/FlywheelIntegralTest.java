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
    public void predictorHoldsSteadySpeedWhenVoltageJustCoversFriction() {
        // At steady state the wheel neither speeds up nor slows down, so the prediction must match.
        StatePredictor p = StatePredictor.create(MechanismModel.flywheel(kV, kA, kS), 0.04);
        p.update(0, 470, kS + kV * 470);
        assertEquals(470, p.velocity(), 0.01);
    }

    @Test
    public void currentLimitHoldsAtStandstill() {
        assertEquals(Flywheel.MAX_MOTOR_AMPS, Flywheel.motorAmps(Flywheel.maxVolts(0), 0), 1e-9);
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
