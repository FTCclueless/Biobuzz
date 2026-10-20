package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class TurretAngleTest {
    private static final double EPS = 1e-9;

    private static double r(double deg) { return Math.toRadians(deg); }

    @Before
    public void limits() {
        Turret.MIN_ANGLE = r(-180);
        Turret.MAX_ANGLE = r(180);
        Turret.LIMIT_MARGIN = r(5);
        Turret.ZERO_POSITION = 0.5;
        Turret.POSITION_PER_RADIAN = 0.001;
    }

    @Test public void servoTravelTightensTheWindow() {
        Turret.MIN_ANGLE = r(-310);
        Turret.MAX_ANGLE = r(50);
        Turret.ZERO_POSITION = 0.5 - 130.0 / 322.0;
        Turret.POSITION_PER_RADIAN = -1.0 / r(322);
        assertEquals(r(-291 + 5), Turret.lowLimit(), 1e-6);
        assertEquals(r(31 - 5), Turret.highLimit(), 1e-6);
        assertEquals(r(26), Turret.chooseReachable(r(40), 0), 1e-6);
    }

    @Test public void plainAngleInRange() {
        assertEquals(r(90), Turret.chooseReachable(r(90), 0), EPS);
    }

    @Test public void wrappedInputIsUnwrapped() {
        assertEquals(r(-90), Turret.chooseReachable(r(270), 0), EPS);
        assertEquals(r(90), Turret.chooseReachable(r(450), 0), EPS);
    }

    @Test public void justBeyondMarginParksAtLimit() {
        assertEquals(r(175), Turret.chooseReachable(r(178), 0), EPS);
        assertEquals(r(-175), Turret.chooseReachable(r(-178), 0), EPS);
    }

    @Test public void crossingTheSeamUnwindsTheLongWay() {
        assertEquals(r(-170), Turret.chooseReachable(r(-170), r(170)), EPS);
    }

    @Test public void neverLeavesTheBand() {
        for (int deg = -720; deg <= 720; deg += 7) {
            for (int cur = -175; cur <= 175; cur += 25) {
                double c = Turret.chooseReachable(r(deg), r(cur));
                if (c < r(-175) - EPS || c > r(175) + EPS) {
                    throw new AssertionError("desired " + deg + " from " + cur + " -> "
                            + Math.toDegrees(c));
                }
            }
        }
    }

    @Test public void narrowRangeParksAtNearerLimit() {
        Turret.MIN_ANGLE = r(-90);
        Turret.MAX_ANGLE = r(90);
        assertEquals(r(85), Turret.chooseReachable(r(120), 0), EPS);
        assertEquals(r(-85), Turret.chooseReachable(r(-120), 0), EPS);
    }

    @Test public void realWindowMinus310ToPlus50() {
        Turret.MIN_ANGLE = r(-310);
        Turret.MAX_ANGLE = r(50);
        assertEquals(r(-90), Turret.chooseReachable(r(-90), 0), EPS);

        assertEquals(r(-180), Turret.chooseReachable(r(180), 0), EPS);

        assertEquals(r(45), Turret.chooseReachable(r(47), 0), EPS);

        assertEquals(r(-305), Turret.chooseReachable(r(-308), r(-250)), EPS);
        for (int deg = -720; deg <= 720; deg += 7) {
            double c = Turret.chooseReachable(r(deg), r(-130));
            if (c < r(-305) - EPS || c > r(45) + EPS) throw new AssertionError(deg + " -> " + Math.toDegrees(c));
        }
    }
}
