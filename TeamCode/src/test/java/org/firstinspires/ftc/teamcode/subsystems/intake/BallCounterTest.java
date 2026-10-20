package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class BallCounterTest {
    private long t = 0;

    private BallCounter counter(int start) { t = 0; return new BallCounter(start, 8, 30, 40); }

    private void hold(BallCounter c, boolean present, int dir, long ms) {
        for (long end = t + ms; t < end; t += 10) c.update(present, dir, t);
    }

    private void pass(BallCounter c, int dir) {
        hold(c, true, dir, 60);
        hold(c, false, dir, 60);
    }

    @Test public void countsBallsComingIn() {
        BallCounter c = counter(0);
        pass(c, +1); pass(c, +1); pass(c, +1);
        assertEquals(3, c.count());
    }

    @Test public void countsOnArrivalNotPassage() {
        BallCounter c = counter(4);
        hold(c, true, +1, 60);
        assertEquals(5, c.count());
    }

    @Test public void rejectingTheBallAtTheWedgeCountsItOut() {
        BallCounter c = counter(4);
        hold(c, true, +1, 60);
        hold(c, true, -1, 60);
        hold(c, false, -1, 60);
        assertEquals(4, c.count());
    }

    @Test public void ejectingABallFromInsideCountsItOut() {
        BallCounter c = counter(3);
        pass(c, -1);
        assertEquals(2, c.count());
    }

    @Test public void bounceShorterThanConfirmIsIgnored() {
        BallCounter c = counter(0);
        hold(c, true, +1, 20);
        hold(c, false, +1, 60);
        assertEquals(0, c.count());
    }

    @Test public void wobbleInFrontOfSensorIsOneBall() {
        BallCounter c = counter(0);
        hold(c, true, +1, 60);
        hold(c, false, +1, 20);
        hold(c, true, +1, 60);
        hold(c, false, +1, 60);
        assertEquals(1, c.count());
    }

    @Test public void ballWaitingWhileStoppedCountsWhenIntakeStarts() {
        BallCounter c = counter(0);
        hold(c, true, 0, 60);
        assertEquals(0, c.count());
        hold(c, true, +1, 10);
        assertEquals(1, c.count());
    }

    @Test public void shotsDecrementAndNeverGoNegative() {
        BallCounter c = counter(2);
        c.shot(1);
        assertEquals(1, c.count());
        c.shot(5);
        assertEquals(0, c.count());
    }

    @Test public void passingInwardDoesNotDoubleCount() {
        BallCounter c = counter(0);
        pass(c, +1);
        assertEquals(1, c.count());
    }

    @Test public void twoEntranceSensorsShareOneCount() {
        t = 0;
        BallCounter c = new BallCounter(2, 0, 8, 30, 40);
        for (int i = 0; i < 2; i++) {
            for (long end = t + 60; t < end; t += 10) { c.update(0, true, +1, t); c.update(1, false, +1, t); }
            for (long end = t + 60; t < end; t += 10) { c.update(0, false, +1, t); c.update(1, true, +1, t); }
            for (long end = t + 60; t < end; t += 10) { c.update(0, false, +1, t); c.update(1, false, +1, t); }
        }
        assertEquals(4, c.count());
    }
}
