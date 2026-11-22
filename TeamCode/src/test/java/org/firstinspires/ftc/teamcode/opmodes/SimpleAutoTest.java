package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.junit.Test;

public class SimpleAutoTest {
    @Test
    public void leaveSpotMovesTowardTheCentreLineForBothAlliances() {
        Pose red = SimpleAuto.leaveSpot(new Pose2d(0, -64, 0), 24); // any distance: the direction is what matters
        assertEquals(-40, red.y(), 1e-9);
        assertEquals(0, red.x(), 1e-9);
        Pose blue = SimpleAuto.leaveSpot(new Pose2d(0, 64, Math.PI), 24);
        assertEquals(40, blue.y(), 1e-9);
    }

    @Test
    public void startPosesPutTheRightSideOnTheWall() {
        double w = Globals.ROBOT_WIDTH / 2, l = Globals.ROBOT_LENGTH / 2;
        Pose2d red = SimpleAuto.startPose(true, false), blue = SimpleAuto.startPose(false, false);
        assertEquals(-72 + w, red.y, 1e-9);
        assertEquals(0, red.heading, 1e-9);
        assertEquals(72 - w, blue.y, 1e-9);
        assertEquals(Math.PI, blue.heading, 1e-9);
        // Loading-zone corners, at each alliance's drivers' left (red -x, blue +x), back on the side wall,
        // and not in the LOADING ZONE (24..48 in from centre along the wall) at the start (G304.E).
        assertEquals(-72 + l, SimpleAuto.startPose(true, true).x, 1e-9);
        assertEquals(72 - l, SimpleAuto.startPose(false, true).x, 1e-9);
        assertTrue(72 - 2 * l > 48);
    }

    @Test
    public void cornerParkEndsInTheLoadingZone() {
        // The whole robot (length along the wall) must stay between the zone's seams, 24..48 in from centre.
        double l = Globals.ROBOT_LENGTH / 2;
        for (boolean red : new boolean[]{true, false}) {
            BiobuzzField.setSide(red ? BiobuzzField.Side.RED : BiobuzzField.Side.BLUE);
            Pose2d s = SimpleAuto.startPose(red, true);
            Pose end = SimpleAuto.overshoot(new Pose(s.x, s.y, s.heading), BiobuzzField.field().park(),
                    SimpleAuto.PARK_EXTRA_IN);
            double ax = Math.abs(end.x());
            assertTrue((red ? "red" : "blue") + " end x " + end.x(), ax - l >= 24 && ax + l <= 48);
        }
        BiobuzzField.setSide(BiobuzzField.Side.RED);
    }

    @Test
    public void parkPosesScoreLeaveAndPark() {
        // LOADING ZONE: 11 in deep off the alliance wall, between the seams 24 and 48 in from the drivers'-left
        // corner (red: x -48..-24, blue: x 24..48). The robot lies along the wall.
        for (BiobuzzField.Side s : BiobuzzField.Side.values()) {
            BiobuzzField.setSide(s);
            Pose p = BiobuzzField.field().park();
            double gap = 72 - Math.abs(p.y()) - Globals.ROBOT_WIDTH / 2; // robot edge to the alliance wall
            assertTrue(s + " touches the wall: no LEAVE", gap > 0.5);
            assertTrue(s + " misses the zone: no PARK", gap < 11 - 2);
            double ax = (s == BiobuzzField.Side.RED ? -1 : 1) * p.x();
            assertTrue(s + " wrong part of the wall", ax > 24 && ax < 48);
        }
        BiobuzzField.setSide(BiobuzzField.Side.RED);
    }

    @Test
    public void parkOvershootsByTheExtraInch() {
        Pose p = SimpleAuto.overshoot(new Pose(0, 0, 0), new Pose(3, 4, 0), 1.0); // 3-4-5 direction
        assertEquals(3.6, p.x(), 1e-9);
        assertEquals(4.8, p.y(), 1e-9);
    }

    @Test
    public void bothShootSpotsAimInsideTheTurretWindow() {
        // The robot's turret (TurretAngleTest changes these statics, so set them here).
        Turret.MIN_ANGLE = Math.toRadians(-180);
        Turret.MAX_ANGLE = Math.toRadians(90);
        Turret.LIMIT_MARGIN = Math.toRadians(5);
        Turret.ZERO_POSITION = 0.697;
        Turret.MAX_POSITION = 0.882;
        Turret.POSITION_PER_RADIAN = 1.0 / Math.toRadians(322);
        for (boolean red : new boolean[]{true, false}) {
            Pose2d start = SimpleAuto.startPose(red, false);
            Pose spot = SimpleAuto.leaveSpot(start, SimpleAuto.LEAVE_DISTANCE);
            Vector3 hive = BiobuzzField.nearestHiveCell(red, new Pose2d(spot.x(), spot.y(), spot.heading()));
            double bearing = Math.atan2(hive.y - spot.y(), hive.x - spot.x());
            double want = Turret.fromFieldBearing(bearing, spot.heading());
            double got = Turret.clipAngle(want);
            assertTrue((red ? "red" : "blue") + " turret clipped: wants " + Math.toDegrees(want),
                    Math.abs(Math.atan2(Math.sin(got - want), Math.cos(got - want))) < 1e-6);
        }
    }
}
