package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.control.LQRController;
import org.firstinspires.ftc.teamcode.control.MechanismModel;
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
}
