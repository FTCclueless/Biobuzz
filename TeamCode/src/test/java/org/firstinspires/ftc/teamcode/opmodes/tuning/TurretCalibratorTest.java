package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class TurretCalibratorTest {
    @Test public void fitRecoversSlopeAndOffset() {
        List<TurretCalibrator.Point> pts = new ArrayList<>();
        for (double pos = 0.5; pos <= 0.85; pos += 0.05) {
            pts.add(new TurretCalibrator.Point(pos, -6.0 * (pos - 0.8) + 0.01, 0));
        }
        double[] fit = TurretCalibrator.fit(pts, 0.8, 1);
        assertEquals(-6.0, fit[0], 1e-9);
        assertEquals(0.01, fit[1], 1e-9);
        assertEquals(0.0, fit[2], 1e-9);
    }

    @Test public void backlashIsTheGapBetweenApproaches() {
        List<TurretCalibrator.Point> pts = new ArrayList<>();
        pts.add(new TurretCalibrator.Point(0.6, -1.00, -1));
        pts.add(new TurretCalibrator.Point(0.6, -0.96, +1));
        pts.add(new TurretCalibrator.Point(0.5, -1.60, -1));
        pts.add(new TurretCalibrator.Point(0.5, -1.54, +1));
        assertEquals(0.05, TurretCalibrator.backlash(pts), 1e-9);
    }
}
