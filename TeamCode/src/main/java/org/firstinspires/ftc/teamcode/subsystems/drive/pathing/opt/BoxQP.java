package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.opt;

import org.firstinspires.ftc.teamcode.utils.MatrixMath;
import org.firstinspires.ftc.teamcode.utils.Utils;

public final class BoxQP {
    public static final class Result {
        public final double[] x;
        public final double cost;
        public final int iterations;
        public final boolean converged;

        Result(double[] x, double cost, int iterations, boolean converged) {
            this.x = x;
            this.cost = cost;
            this.iterations = iterations;
            this.converged = converged;
        }
    }

    private double omega = 1.4;
    private int maxIterations = 600;
    private double tolerance = 1e-9;

    public BoxQP omega(double v) { this.omega = v; return this; }

    public BoxQP maxIterations(int v) { this.maxIterations = v; return this; }

    public BoxQP tolerance(double v) { this.tolerance = v; return this; }

    public Result solve(double[][] H, double[] f, double[] lo, double[] hi, double[] x0) {
        int n = f.length;
        double[] x = new double[n];
        for (int i = 0; i < n; i++) {
            if (lo[i] > hi[i]) {
                throw new IllegalArgumentException("Empty box at index " + i);
            }
            x[i] = Utils.minMaxClip(x0 == null ? 0 : x0[i], lo[i], hi[i]);
        }

        boolean converged = false;
        int iter = 0;
        for (; iter < maxIterations; iter++) {
            double maxStep = 0;
            for (int i = 0; i < n; i++) {
                double hii = H[i][i];
                double xi;
                if (hii <= 1e-12) {
                    xi = f[i] > 0 ? lo[i] : (f[i] < 0 ? hi[i] : x[i]);
                } else {
                    double g = MatrixMath.rowDot(H, i, x, f[i]);
                    xi = Utils.minMaxClip(x[i] - omega * g / hii, lo[i], hi[i]);
                }
                maxStep = Math.max(maxStep, Math.abs(xi - x[i]));
                x[i] = xi;
            }
            if (maxStep < tolerance) {
                converged = true;
                iter++;
                break;
            }
        }
        return new Result(x, cost(H, f, x), iter, converged);
    }

    public static double cost(double[][] H, double[] f, double[] x) {
        return 0.5 * MatrixMath.quadratic(H, x) + MatrixMath.dot(f, x);
    }

    public static double projectedGradientNorm(double[][] H, double[] f, double[] x,
                                               double[] lo, double[] hi) {
        double worst = 0;
        for (int i = 0; i < x.length; i++) {
            double g = MatrixMath.rowDot(H, i, x, f[i]);
            double pg = g;
            if (x[i] <= lo[i] + 1e-12) pg = Math.min(0, g);
            if (x[i] >= hi[i] - 1e-12) pg = Math.max(0, g);
            worst = Math.max(worst, Math.abs(pg));
        }
        return worst;
    }
}
