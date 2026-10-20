package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class SweepSelectorTest {
    private static List<Boolean> p(Boolean... b) { return Arrays.asList(b); }

    private static List<Double> s(Double... d) { return Arrays.asList(d); }

    @Test public void nonePassed() {
        assertEquals(-1, SweepSelector.choose(p(false, false), s(Double.NaN, 1.0)));
    }

    @Test public void fastestWhenNotAtEdge() {
        assertEquals(2, SweepSelector.choose(p(true, true, true), s(0.9, 0.6, 0.4)));
    }

    @Test public void backsOffOneStepAtTheEdge() {
        assertEquals(1, SweepSelector.choose(p(true, true, true, false), s(0.9, 0.6, 0.4, 0.3)));
    }

    @Test public void keepsEdgeWinnerIfGentlerNeighbourFailed() {
        assertEquals(1, SweepSelector.choose(p(false, true, false), s(Double.NaN, 0.5, 0.3)));
    }

    @Test public void earlyFailuresFromBeingTooGentleAreIgnored() {
        assertEquals(3, SweepSelector.choose(p(false, false, true, true), s(Double.NaN, Double.NaN, 0.7, 0.5)));
    }
}
