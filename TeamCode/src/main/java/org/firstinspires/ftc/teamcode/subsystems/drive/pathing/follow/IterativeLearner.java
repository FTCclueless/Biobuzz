package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow;

import org.firstinspires.ftc.teamcode.utils.Utils;

public final class IterativeLearner {
    private final double[] table;
    private final double[] accum;
    private final double[] weight;
    private final double cellSize;
    private final double length;

    private double gamma = 0.5;
    private double maxDelta = 6.0;
    private double maxMagnitude = 25.0;
    private int smoothingPasses = 2;
    private int runs = 0;

    public IterativeLearner(double pathLength, double cellSize) {
        this.length = pathLength;
        this.cellSize = Math.max(0.5, cellSize);
        int n = Math.max(2, (int) Math.ceil(pathLength / this.cellSize) + 1);
        this.table = new double[n];
        this.accum = new double[n];
        this.weight = new double[n];
    }

    public IterativeLearner(double pathLength) {
        this(pathLength, 2.0);
    }

    public IterativeLearner gamma(double v) { this.gamma = Utils.minMaxClip(v, 0, 1); return this; }

    public IterativeLearner maxDelta(double v) { this.maxDelta = v; return this; }

    public IterativeLearner maxMagnitude(double v) { this.maxMagnitude = v; return this; }

    public IterativeLearner smoothingPasses(int v) { this.smoothingPasses = v; return this; }

    public int runCount() { return runs; }

    public void startRun() {
        java.util.Arrays.fill(accum, 0);
        java.util.Arrays.fill(weight, 0);
    }

    public double correctionAt(double s) {
        double c = s / cellSize;
        int i = Utils.minMaxClip((int) Math.floor(c), 0, table.length - 1);
        int j = Utils.minMaxClip(i + 1, 0, table.length - 1);
        double f = Utils.minMaxClip(c - i, 0, 1);
        return table[i] + (table[j] - table[i]) * f;
    }

    public void observe(double s, double feedbackLateral, double dt) {
        if (Double.isNaN(feedbackLateral)) return;
        int i = Utils.minMaxClip((int) Math.round(s / cellSize), 0, table.length - 1);
        accum[i] += feedbackLateral * dt;
        weight[i] += dt;
    }

    public void endRun() {
        double[] delta = new double[table.length];
        for (int i = 0; i < table.length; i++) {
            if (weight[i] < 1e-6) continue;
            double meanFeedback = accum[i] / weight[i];
            delta[i] = Utils.minMaxClip(gamma * meanFeedback, -maxDelta, maxDelta);
        }

        for (int pass = 0; pass < smoothingPasses; pass++) {
            double[] sm = new double[delta.length];
            for (int i = 0; i < delta.length; i++) {
                double a = delta[Math.max(0, i - 1)];
                double b = delta[i];
                double c = delta[Math.min(delta.length - 1, i + 1)];
                sm[i] = 0.25 * a + 0.5 * b + 0.25 * c;
            }
            delta = sm;
        }

        for (int i = 0; i < table.length; i++) {
            table[i] = Utils.minMaxClip(table[i] + delta[i], -maxMagnitude, maxMagnitude);
        }
        runs++;
    }

    public double[] getTable() { return table.clone(); }

    public void setTable(double[] t) {
        if (t.length != table.length) {
            throw new IllegalArgumentException(
                    "Table size mismatch: expected " + table.length + ", got " + t.length
                            + " (was it learned on a different path?)");
        }
        System.arraycopy(t, 0, table, 0, t.length);
    }

    public double cellSize() { return cellSize; }

    public int size() { return table.length; }

    public double pathLength() { return length; }

    public double peakCorrection() {
        double m = 0;
        for (double v : table) m = Math.max(m, Math.abs(v));
        return m;
    }

    public void clear() {
        java.util.Arrays.fill(table, 0);
        startRun();
        runs = 0;
    }
}
