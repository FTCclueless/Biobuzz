package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class FlywheelModelTest {
    private static FlywheelProcedures.Model model(double... velocities) {
        FlywheelProcedures.Model m = new FlywheelProcedures.Model();
        for (int i = 0; i < velocities.length; i++) {
            FlywheelProcedures.Point p = new FlywheelProcedures.Point();
            p.power = 0.35 + 0.15 * i;
            p.velocity = velocities[i];
            p.volts = 0.35 + 0.0155 * p.velocity; // a real motor: kS + kV * speed
            m.points.add(p);
        }
        double[] fit = FlywheelProcedures.fitLine(m.points);
        m.kV = fit[0];
        m.kS = fit[1];
        m.tau = 0.2;
        m.kA = m.kV * m.tau;
        return m;
    }

    @Test
    public void stalledWheelIsRejectedEvenWhenTheFitLooksPositive() {
        // Noise around 0-4 in/s: kV comes out positive, which the old check accepted.
        assertNotNull(model(0.5, 1.0, 2.0, 3.0, 4.5).problem());
    }

    @Test
    public void slowTopSpeedIsRejected() {
        assertNotNull(model(10, 25, 40, 60, 80).problem());
    }

    @Test
    public void healthyWheelPasses() {
        assertNull(model(240, 330, 420, 500, 580).problem());
    }

    @Test
    public void pointOffTheLineIsRejected() {
        FlywheelProcedures.Model m = model(240, 330, 420, 500, 580);
        m.points.get(2).volts += 1.5; // one unsteady step (e.g. a brownout mid-hold)
        double[] fit = FlywheelProcedures.fitLine(m.points);
        m.kV = fit[0];
        m.kS = fit[1];
        assertNotNull(m.problem());
    }
}
