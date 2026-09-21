package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

import org.firstinspires.ftc.teamcode.utils.MatrixMath;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;

public final class SplineFitter {
    private SplineFitter() {}

    public static final class Spline {
        public final Vector2[] pts;
        public final double[] h;
        public final double[] mx;
        public final double[] my;
        private final double[][] mMatrix;
        private final double[] cx;
        private final double[] cy;

        Spline(Vector2[] pts, double[] h, double[][] mMatrix, double[] cx, double[] cy,
               double[] mx, double[] my) {
            this.pts = pts;
            this.h = h;
            this.mMatrix = mMatrix;
            this.cx = cx;
            this.cy = cy;
            this.mx = mx;
            this.my = my;
        }

        public int knotCount() { return pts.length; }

        public double[][] secondDerivMap() { return mMatrix; }

        public double[] constX() { return cx; }

        public double[] constY() { return cy; }

        public Vector2 knotDeriv(int i) {
            int n = pts.length - 1;
            if (i < n) {
                double bx = (pts[i + 1].x - pts[i].x) / h[i] - h[i] * (2 * mx[i] + mx[i + 1]) / 6.0;
                double by = (pts[i + 1].y - pts[i].y) / h[i] - h[i] * (2 * my[i] + my[i + 1]) / 6.0;
                return new Vector2(bx, by);
            }
            int j = n - 1;
            double bx = (pts[j + 1].x - pts[j].x) / h[j] - h[j] * (2 * mx[j] + mx[j + 1]) / 6.0;
            double by = (pts[j + 1].y - pts[j].y) / h[j] - h[j] * (2 * my[j] + my[j + 1]) / 6.0;
            return new Vector2(bx + h[j] * (mx[j] + mx[j + 1]) / 2.0,
                            by + h[j] * (my[j] + my[j + 1]) / 2.0);
        }

        public double knotCurvature(int i) {
            Vector2 d = knotDeriv(i);
            double sp = d.mag();
            if (sp < 1e-9) return 0;
            return (d.x * my[i] - d.y * mx[i]) / (sp * sp * sp);
        }

        public Path toPath() {
            List<BezierSegment> segs = new ArrayList<BezierSegment>(pts.length - 1);
            for (int i = 0; i < pts.length - 1; i++) {
                double hi = h[i];
                double bx = (pts[i + 1].x - pts[i].x) / hi - hi * (2 * mx[i] + mx[i + 1]) / 6.0;
                double by = (pts[i + 1].y - pts[i].y) / hi - hi * (2 * my[i] + my[i + 1]) / 6.0;
                double ex = bx + hi * (mx[i] + mx[i + 1]) / 2.0;
                double ey = by + hi * (my[i] + my[i + 1]) / 2.0;
                Vector2 p0 = pts[i];
                Vector2 p3 = pts[i + 1];
                Vector2 p1 = new Vector2(p0.x + hi * bx / 3.0, p0.y + hi * by / 3.0);
                Vector2 p2 = new Vector2(p3.x - hi * ex / 3.0, p3.y - hi * ey / 3.0);
                segs.add(new BezierSegment(p0, p1, p2, p3));
            }
            return new Path(segs);
        }
    }

    public static Spline fit(Vector2[] pts) {
        return fit(pts, null, null);
    }

    public static Path fitPath(Vector2[] pts) {
        return fit(pts).toPath();
    }

    public static Spline fit(Vector2[] pts, Vector2 startTangent, Vector2 endTangent) {
        int n = pts.length - 1;
        if (n < 1) throw new IllegalArgumentException("Need at least 2 points to fit a spline");

        double[] h = new double[n];
        for (int i = 0; i < n; i++) {
            h[i] = Vector2.distance(pts[i + 1], pts[i]);
            if (h[i] < 1e-6) {
                throw new IllegalArgumentException(
                        "Duplicate/near-duplicate points at index " + i + " -- spline is undefined");
            }
        }

        int N = n + 1;
        double[] sub = new double[N];
        double[] diag = new double[N];
        double[] sup = new double[N];
        double[][] B = new double[N][N];
        double[] cRawX = new double[N];
        double[] cRawY = new double[N];

        for (int i = 1; i < N - 1; i++) {
            sub[i] = h[i - 1];
            diag[i] = 2.0 * (h[i - 1] + h[i]);
            sup[i] = h[i];
            B[i][i - 1] = 6.0 / h[i - 1];
            B[i][i] = -6.0 / h[i - 1] - 6.0 / h[i];
            B[i][i + 1] = 6.0 / h[i];
        }

        if (startTangent == null) {
            diag[0] = 1.0;
        } else {
            diag[0] = 2.0 * h[0];
            sup[0] = h[0];
            B[0][0] = -6.0 / h[0];
            B[0][1] = 6.0 / h[0];
            cRawX[0] = -6.0 * startTangent.x;
            cRawY[0] = -6.0 * startTangent.y;
        }

        if (endTangent == null) {
            diag[N - 1] = 1.0;
        } else {
            sub[N - 1] = h[n - 1];
            diag[N - 1] = 2.0 * h[n - 1];
            B[N - 1][N - 2] = 6.0 / h[n - 1];
            B[N - 1][N - 1] = -6.0 / h[n - 1];
            cRawX[N - 1] = 6.0 * endTangent.x;
            cRawY[N - 1] = 6.0 * endTangent.y;
        }

        double[][] M = MatrixMath.tridiagonalSolveMatrix(sub, diag, sup, B);
        double[] cx = new double[N];
        double[] cy = new double[N];
        MatrixMath.tridiagonal(sub, diag, sup, cRawX, cx);
        MatrixMath.tridiagonal(sub, diag, sup, cRawY, cy);

        double[] px = new double[N];
        double[] py = new double[N];
        for (int i = 0; i < N; i++) {
            px[i] = pts[i].x;
            py[i] = pts[i].y;
        }
        double[] mx = MatrixMath.matVecAdd(M, px, cx);
        double[] my = MatrixMath.matVecAdd(M, py, cy);

        return new Spline(pts.clone(), h, M, cx, cy, mx, my);
    }

    public static Vector2[] resample(Path path, int count) {
        if (count < 3) count = 3;
        Vector2[] out = new Vector2[count];
        for (int i = 0; i < count; i++) {
            out[i] = path.point(path.length() * i / (count - 1.0));
        }
        return out;
    }
}
