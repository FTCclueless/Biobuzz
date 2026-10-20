package org.firstinspires.ftc.teamcode.subsystems.intake;

public class BallCounter {
    private final long minPresentMs;
    private final long minGapMs;
    private final int maxCount;

    private int count;
    private final boolean[] confirmed;
    private final boolean[] countedIn;
    private final long[] presentSince;
    private final long[] absentSince;

    public BallCounter(int startCount, int maxCount, long minPresentMs, long minGapMs) {
        this(1, startCount, maxCount, minPresentMs, minGapMs);
    }

    public BallCounter(int sensors, int startCount, int maxCount, long minPresentMs, long minGapMs) {
        this.count = Math.max(0, Math.min(maxCount, startCount));
        this.maxCount = maxCount;
        this.minPresentMs = minPresentMs;
        this.minGapMs = minGapMs;
        confirmed = new boolean[sensors];
        countedIn = new boolean[sensors];
        presentSince = new long[sensors];
        absentSince = new long[sensors];
        java.util.Arrays.fill(presentSince, -1);
        java.util.Arrays.fill(absentSince, -1);
    }

    public int update(boolean present, int direction, long nowMs) {
        return update(0, present, direction, nowMs);
    }

    public int update(int sensor, boolean present, int direction, long nowMs) {
        if (present) {
            absentSince[sensor] = -1;
            if (presentSince[sensor] < 0) presentSince[sensor] = nowMs;
            if (!confirmed[sensor] && nowMs - presentSince[sensor] >= minPresentMs) confirmed[sensor] = true;
            if (confirmed[sensor] && !countedIn[sensor] && direction > 0) {
                countedIn[sensor] = true;
                return add(+1);
            }
        } else {
            presentSince[sensor] = -1;
            if (absentSince[sensor] < 0) absentSince[sensor] = nowMs;
            if (confirmed[sensor] && nowMs - absentSince[sensor] >= minGapMs) {
                confirmed[sensor] = false;
                countedIn[sensor] = false;
                if (direction < 0) return add(-1);
            }
        }
        return 0;
    }

    public void shot(int n) {
        for (int i = 0; i < n; i++) add(-1);
    }

    private int add(int d) {
        int before = count;
        count = Math.max(0, Math.min(maxCount, count + d));
        return count - before;
    }

    public int count() { return count; }

    public void set(int n) { count = Math.max(0, Math.min(maxCount, n)); }
}
