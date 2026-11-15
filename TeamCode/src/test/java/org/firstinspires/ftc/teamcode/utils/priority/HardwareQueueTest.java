package org.firstinspires.ftc.teamcode.utils.priority;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.junit.Test;

public class HardwareQueueTest {
    /** Minimal motor-like device: a pending value that the queue writes when it picks it. */
    static class FakeDevice extends PriorityDevice {
        double target, written;
        int writes;

        FakeDevice(String name) { super(3, 5, name); callLengthMillis = 1.6; }

        @Override protected double commandedValue() { return target; }
        @Override protected boolean hasPendingWrite() { return target != written; }
        @Override protected double error() { return Math.abs(target - written); }
        @Override protected double getPriority(double timeRemaining) {
            if (!hasPendingWrite()) return 0;
            if (timeRemaining * 1000.0 <= costMillis()) return 0;
            return rank();
        }
        @Override protected void update() { written = target; writes++; }
    }

    @Test
    public void overBudgetLoopStillWritesOneDevicePerLoop() {
        HardwareQueue q = new HardwareQueue();
        FakeDevice a = new FakeDevice("a"), b = new FakeDevice("b");
        q.addDevices(a, b);
        a.target = 1;
        b.target = 1;

        Globals.LOOP_START = System.nanoTime() - 100_000_000L; // loop already 100 ms over
        q.update();
        assertEquals(1, a.writes + b.writes);

        q.update();
        assertEquals("the other device gets the next loop's write", 2, a.writes + b.writes);
        assertEquals(1, a.writes);
        assertEquals(1, b.writes);
    }

    @Test
    public void underBudgetLoopWritesEverythingPending() {
        HardwareQueue q = new HardwareQueue();
        FakeDevice a = new FakeDevice("a"), b = new FakeDevice("b");
        q.addDevices(a, b);
        a.target = 1;
        b.target = 1;

        Globals.START_LOOP();
        q.update();
        assertEquals(1, a.writes);
        assertEquals(1, b.writes);
    }
}
