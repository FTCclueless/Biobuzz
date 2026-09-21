package org.firstinspires.ftc.teamcode.utils.priority;

public abstract class PriorityDevice {
    protected final double consequence;
    protected final double growthWeight;

    public final String name;

    protected double lastUpdateTime;
    protected double callLengthMillis;
    protected int actuatorCount = 1;

    private boolean critical = false;
    private double maxStaleness = Double.MAX_VALUE;

    private long lastAdvanceNanos = System.nanoTime();
    private double lastSeenCommand = 0;
    private double growth = 0;

    boolean isUpdated = false;

    public PriorityDevice(double basePriority, double priorityScale, String name) {
        this.consequence = basePriority;
        this.growthWeight = priorityScale;
        this.name = name;
        lastUpdateTime = System.nanoTime();
    }

    public PriorityDevice setCritical(boolean v) { this.critical = v; return this; }

    public boolean isCritical() { return critical; }

    public boolean needsCriticalWrite() {
        return hasPendingWrite() || stalenessSeconds() >= maxStaleness;
    }

    public PriorityDevice setMaxStaleness(double seconds) { this.maxStaleness = seconds; return this; }

    public double costMillis() { return Math.max(callLengthMillis * actuatorCount, 1e-3); }

    protected double stalenessSeconds() { return (System.nanoTime() - lastUpdateTime) / 1.0E9; }

    protected double growth() { return growth; }

    public final void advanceModel() {
        long now = System.nanoTime();
        double dt = (now - lastAdvanceNanos) / 1.0E9;
        lastAdvanceNanos = now;
        if (dt <= 0) return;

        onAdvance(dt);

        if (dt >= 1.0E-3) {
            double c = commandedValue();
            growth = Math.abs(c - lastSeenCommand) / dt;
            lastSeenCommand = c;
        }
    }

    protected void onAdvance(double dt) {}

    protected abstract double commandedValue();

    protected abstract boolean hasPendingWrite();

    protected abstract double error();

    protected double rank() {
        double staleness = stalenessSeconds();
        if (staleness >= maxStaleness) return Double.MAX_VALUE;

        double harm = consequence * error() * staleness
                    + growthWeight * 0.5 * growth * staleness * staleness;
        return harm / costMillis();
    }

    protected abstract double getPriority(double timeRemaining);

    protected abstract void update();

    public void resetUpdateBoolean() {
        isUpdated = false;
    }
}
