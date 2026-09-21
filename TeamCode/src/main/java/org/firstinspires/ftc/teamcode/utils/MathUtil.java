package org.firstinspires.ftc.teamcode.utils;

public final class MathUtil {
    private static final double EPS = 1e-9;

    private MathUtil() {}

    public static double sq(double v) { return v * v; }

    public static double safeSqrt(double v) {
        return v <= 0 ? 0 : Math.sqrt(v);
    }

    public static int gridNodes(double length, double resolution, int minNodes) {
        if (!(length > 0) || !(resolution > 0)) return Math.max(2, minNodes);
        return Math.max(Math.max(2, minNodes), (int) Math.ceil(length / resolution) + 1);
    }

    public interface Curve {
        double at(double s);
    }

    public static double derivative(Curve f, double s, double step, double lo, double hi) {
        double a = Utils.minMaxClip(s - step, lo, hi);
        double b = Utils.minMaxClip(s + step, lo, hi);
        double span = b - a;
        if (span < 1e-12) return 0;
        return (f.at(b) - f.at(a)) / span;
    }

    public static double angleDerivative(Curve f, double s, double step, double lo, double hi) {
        double a = Utils.minMaxClip(s - step, lo, hi);
        double b = Utils.minMaxClip(s + step, lo, hi);
        double span = b - a;
        if (span < 1e-12) return 0;
        return Utils.headingClip(f.at(b) - f.at(a)) / span;
    }

    public static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    public static boolean allFinite(double... vs) {
        for (double v : vs) {
            if (!isFinite(v)) return false;
        }
        return true;
    }

    public static int lowerBound(double[] xs, double x) {
        if (xs.length < 2) return 0;
        int lo = 0, hi = xs.length - 1;
        if (x <= xs[0]) return 0;
        if (x >= xs[hi]) return hi - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (xs[mid] <= x) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    public static double interp(double[] xs, double[] ys, double x) {
        if (xs.length == 1) return ys[0];
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        int i = lowerBound(xs, x);
        double dx = xs[i + 1] - xs[i];
        if (dx < EPS) return ys[i];
        return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / dx;
    }

    public static double interpAngle(double[] xs, double[] ys, double x) {
        if (xs.length == 1) return ys[0];
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        int i = lowerBound(xs, x);
        double dx = xs[i + 1] - xs[i];
        if (dx < EPS) return ys[i];
        double t = (x - xs[i]) / dx;
        return ys[i] + Utils.headingClip(ys[i + 1] - ys[i]) * t;
    }
}
