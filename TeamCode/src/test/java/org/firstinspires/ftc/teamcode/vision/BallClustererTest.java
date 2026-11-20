package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BallClustererTest {
    private static BallClusterer.Ball pollen(double x, double y) {
        return new BallClusterer.Ball(new Vector2(x, y), BallClusterer.Type.POLLEN);
    }

    private static BallClusterer.Ball nectar(double x, double y, boolean red) {
        return new BallClusterer.Ball(new Vector2(x, y),
                red ? BallClusterer.Type.RED_NECTAR : BallClusterer.Type.BLUE_NECTAR);
    }

    @Test
    public void picksBiggestClusterOverCloserSmallOne() {
        BallClusterer c = new BallClusterer();
        List<BallClusterer.Ball> balls = Arrays.asList(
                pollen(10, 0),                                     // 1 ball, close
                pollen(40, 20), pollen(43, 21), pollen(41, 24));   // 3 balls, far
        BallClusterer.Cluster best = c.best(balls, new Vector2(0, 0));
        assertNotNull(best);
        assertEquals(3, best.balls.size());
        assertEquals(41.33, best.centroid.x, 0.01);
    }

    @Test
    public void chainsTouchingBallsIntoOneCluster() {
        BallClusterer c = new BallClusterer();
        List<BallClusterer.Ball> line = new ArrayList<>();
        for (int i = 0; i < 5; i++) line.add(pollen(20 + i * 5, 0)); // 5 in apart, linkDist 7
        List<BallClusterer.Cluster> clusters = c.cluster(line, new Vector2(0, 0));
        assertEquals(1, clusters.size());
        assertEquals(5, clusters.get(0).balls.size());
    }

    @Test
    public void ignoresOpponentNectarAndOffField() {
        BallClusterer c = new BallClusterer();
        c.isRed = true;
        List<BallClusterer.Ball> balls = Arrays.asList(
                nectar(30, 0, false), nectar(32, 0, false), nectar(34, 0, false), // opponent
                pollen(0, 75),                                                    // outside the wall
                nectar(-20, 0, true));                                            // ours
        BallClusterer.Cluster best = c.best(balls, new Vector2(0, 0));
        assertNotNull(best);
        assertEquals(1, best.balls.size());
        assertEquals(BallClusterer.Type.RED_NECTAR, best.balls.get(0).type);
    }

    @Test
    public void hysteresisKeepsCurrentTargetOnNearTie() {
        BallClusterer c = new BallClusterer();
        List<BallClusterer.Ball> a = Arrays.asList(pollen(30, 30), pollen(33, 30));
        List<BallClusterer.Ball> both = new ArrayList<>(a);
        both.addAll(Arrays.asList(pollen(30, -30), pollen(33, -30)));
        BallClusterer.Cluster first = c.best(a, new Vector2(0, 0));
        // Second pile is identical (and robot drifted slightly toward it): don't switch.
        BallClusterer.Cluster second = c.best(both, new Vector2(0, -1));
        assertEquals(first.centroid.y, second.centroid.y, 1e-9);
        // A clearly bigger pile does win.
        both.add(pollen(31, -27));
        both.add(pollen(31, -33));
        BallClusterer.Cluster third = c.best(both, new Vector2(0, -1));
        assertTrue(third.centroid.y < 0);
    }

    @Test
    public void approachPoseIsShortOfCentroidFacingIt() {
        BallClusterer c = new BallClusterer();
        c.approachStandoff = 10;
        BallClusterer.Target t = c.target(Arrays.asList(pollen(50, 0), pollen(52, 0)), new Vector2(0, 0));
        assertEquals(41, t.approach.x, 1e-9);
        assertEquals(0, t.approach.y, 1e-9);
        assertEquals(0, t.heading, 1e-9);
    }

    @Test
    public void emptyGivesNull() {
        assertNull(new BallClusterer().target(new ArrayList<>(), new Vector2(0, 0)));
    }

    @Test
    public void groundProjectionStraightAhead() {
        // Camera 10 in up, pitched 20 deg down, target on the crosshair, ball center 1.4 in up:
        // distance = (10 - 1.4) / tan(20 deg) = 23.63 in ahead of the lens.
        Vector2 p = GroundProjection.toRobotFrame(0, 0, 8, 0, 10, 20, 0, 1.4, 200);
        assertNotNull(p);
        assertEquals(8 + 8.6 / Math.tan(Math.toRadians(20)), p.x, 1e-6);
        assertEquals(0, p.y, 1e-9);
    }

    @Test
    public void groundProjectionRightIsNegativeY() {
        Vector2 p = GroundProjection.toRobotFrame(15, -5, 0, 0, 10, 20, 0, 1.4, 200);
        assertNotNull(p);
        assertTrue(p.y < 0);
        // Above the horizon never hits the floor.
        assertNull(GroundProjection.toRobotFrame(0, 25, 0, 0, 10, 20, 0, 1.4, 200));
    }

    @Test
    public void trackerNeedsTwoHitsAndTimesOut() {
        BallTracker t = new BallTracker();
        List<BallClusterer.Ball> frame = Arrays.asList(pollen(10, 10));
        t.update(frame, 0);
        assertEquals(0, t.confirmed().size());
        t.update(Arrays.asList(pollen(10.5, 10)), 50);
        assertEquals(1, t.confirmed().size());
        t.prune(50 + t.timeoutMs + 1);
        assertEquals(0, t.confirmed().size());
    }

    @Test
    public void trackerKeepsTypesSeparate() {
        BallTracker t = new BallTracker();
        t.update(Arrays.asList(pollen(0, 0), nectar(0.5, 0, true)), 0);
        assertEquals(2, t.allTracks().size());
        BallTracker.Track first = t.allTracks().get(0);
        t.update(Arrays.asList(pollen(0.2, 0)), 10);
        assertSame(first, t.allTracks().get(0));
        assertEquals(2, first.hits);
    }
}
