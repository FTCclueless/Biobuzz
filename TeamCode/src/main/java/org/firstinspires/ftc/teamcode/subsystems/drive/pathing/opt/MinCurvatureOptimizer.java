package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.opt;

import org.firstinspires.ftc.teamcode.utils.MatrixMath;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.SplineFitter;

public final class MinCurvatureOptimizer {
    public static final class Result {
        public final Vector2[] points;
        public final double[] alphas;
        public final double initialCost;
        public final double finalCost;
        public final int iterations;
        public final int acceptedSteps;
        public final int rejectedSteps;

        Result(Vector2[] points, double[] alphas, double initialCost, double finalCost,
               int iterations, int accepted, int rejected) {
            this.points = points;
            this.alphas = alphas;
            this.initialCost = initialCost;
            this.finalCost = finalCost;
            this.iterations = iterations;
            this.acceptedSteps = accepted;
            this.rejectedSteps = rejected;
        }

        public double improvement() {
            return initialCost <= 0 ? 0 : 1.0 - finalCost / initialCost;
        }

        public Path toPath() {
            return SplineFitter.fit(points).toPath();
        }
    }

    private int maxIterations = 12;
    private double initialTrustRadius = 1.5;
    private double minTrustRadius = 0.02;
    private double trustGrow = 1.6;
    private double trustShrink = 0.5;
    private double ridge = 1e-4;
    private final BoxQP qp = new BoxQP();

    public MinCurvatureOptimizer maxIterations(int v) { this.maxIterations = v; return this; }

    public MinCurvatureOptimizer trustRadius(double v) { this.initialTrustRadius = v; return this; }

    public MinCurvatureOptimizer ridge(double v) { this.ridge = v; return this; }

    public Result optimize(Vector2[] seed, Vector2[] normals, double[] lo, double[] hi) {
        int n = seed.length;
        if (normals.length != n || lo.length != n || hi.length != n) {
            throw new IllegalArgumentException("Seed, normals and bounds must be the same length");
        }

        double[] alpha = new double[n];
        Vector2[] pts = apply(seed, normals, alpha);
        double bestCost = trueCurvatureCost(pts);
        double initialCost = bestCost;

        double delta = initialTrustRadius;
        int accepted = 0, rejected = 0, iter = 0;

        for (; iter < maxIterations && delta > minTrustRadius; iter++) {
            SplineFitter.Spline sp;
            try {
                sp = SplineFitter.fit(pts);
            } catch (IllegalArgumentException e) {
                break;
            }
            double[][] M = sp.secondDerivMap();
            double[] cx = sp.constX();
            double[] cy = sp.constY();

            double[] wx = new double[n];
            double[] wy = new double[n];
            for (int i = 0; i < n; i++) {
                Vector2 d = sp.knotDeriv(i);
                double sp2 = d.mag2();
                double q = Math.pow(Math.max(sp2, 1e-9), 1.5);
                wx[i] = d.x / q;
                wy[i] = d.y / q;
            }

            double[] px = new double[n];
            double[] py = new double[n];
            double[] nx = new double[n];
            double[] ny = new double[n];
            for (int i = 0; i < n; i++) {
                px[i] = pts[i].x;
                py[i] = pts[i].y;
                nx[i] = normals[i].x;
                ny[i] = normals[i].y;
            }
            double[] mx = MatrixMath.matVecAdd(M, px, cx);
            double[] my = MatrixMath.matVecAdd(M, py, cy);
            double[] kCur = new double[n];
            for (int i = 0; i < n; i++) {
                kCur[i] = wx[i] * my[i] - wy[i] * mx[i];
            }
            double[][] T = MatrixMath.subtract(
                    MatrixMath.rowColumnScaled(M, wx, ny),
                    MatrixMath.rowColumnScaled(M, wy, nx));

            double[][] H = MatrixMath.scale(MatrixMath.gramian(T), 2.0);
            double[] tk = MatrixMath.transposeMatVec(T, kCur);
            double[] f = new double[n];
            for (int i = 0; i < n; i++) {
                H[i][i] += 2.0 * ridge;
                f[i] = 2.0 * (tk[i] + ridge * alpha[i]);
            }

            double[] tlo = new double[n];
            double[] thi = new double[n];
            for (int i = 0; i < n; i++) {
                tlo[i] = Math.max(lo[i], alpha[i] - delta) - alpha[i];
                thi[i] = Math.min(hi[i], alpha[i] + delta) - alpha[i];
                if (tlo[i] > thi[i]) {
                    tlo[i] = thi[i] = 0;
                }
            }

            BoxQP.Result step = qp.solve(H, f, tlo, thi, null);
            double[] cand = new double[n];
            for (int i = 0; i < n; i++) {
                cand[i] = Utils.minMaxClip(alpha[i] + step.x[i], lo[i], hi[i]);
            }
            Vector2[] candidate = apply(seed, normals, cand);

            double candCost;
            try {
                candCost = trueCurvatureCost(candidate);
            } catch (IllegalArgumentException e) {
                candCost = Double.MAX_VALUE;
            }

            if (candCost < bestCost - 1e-12) {
                alpha = cand;
                pts = candidate;
                bestCost = candCost;
                accepted++;
                delta = Math.min(delta * trustGrow, initialTrustRadius * 4);
            } else {
                rejected++;
                delta *= trustShrink;
            }
        }

        return new Result(pts, alpha, initialCost, bestCost, iter, accepted, rejected);
    }

    private static Vector2[] apply(Vector2[] seed, Vector2[] normals, double[] alpha) {
        Vector2[] out = new Vector2[seed.length];
        for (int i = 0; i < seed.length; i++) {
            out[i] = seed[i].plus(normals[i].times(alpha[i]));
        }
        return out;
    }

    public static double trueCurvatureCost(Vector2[] pts) {
        SplineFitter.Spline sp = SplineFitter.fit(pts);
        double total = 0;
        int n = pts.length;
        for (int i = 0; i < n; i++) {
            double k = sp.knotCurvature(i);
            double w;
            if (i == 0) w = 0.5 * sp.h[0];
            else if (i == n - 1) w = 0.5 * sp.h[n - 2];
            else w = 0.5 * (sp.h[i - 1] + sp.h[i]);
            total += k * k * w;
        }
        return total;
    }

    public static double peakCurvature(Vector2[] pts) {
        SplineFitter.Spline sp = SplineFitter.fit(pts);
        double peak = 0;
        for (int i = 0; i < pts.length; i++) {
            peak = Math.max(peak, Math.abs(sp.knotCurvature(i)));
        }
        return peak;
    }
}
