package org.firstinspires.ftc.teamcode.utils;

public final class RateEstimator {
    private double lastValue = 0;
    private boolean haveBaseline = false;
    private double lastRate = 0;

    public double update(double value, double dt) {
        if (!haveBaseline || dt <= 1e-9) {
            lastValue = value;
            haveBaseline = true;
            lastRate = 0;
            return 0;
        }
        lastRate = (value - lastValue) / dt;
        lastValue = value;
        return lastRate;
    }

    public void observeOnly(double value) {
        lastValue = value;
        haveBaseline = true;
        lastRate = 0;
    }

    public void invalidate() {
        haveBaseline = false;
        lastRate = 0;
    }

    public boolean hasBaseline() { return haveBaseline; }

    public double rate() { return lastRate; }

    public double lastValue() { return lastValue; }

    public void reset() {
        lastValue = 0;
        lastRate = 0;
        haveBaseline = false;
    }
}
