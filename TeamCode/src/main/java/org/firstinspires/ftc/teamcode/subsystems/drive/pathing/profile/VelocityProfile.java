package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.List;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;

public final class VelocityProfile {
    private static final int CURVATURE_SUBSAMPLES = 4;

    private static final int MIN_NODES = 16;

    private final double[] s;
    private final double[] v;
    private final double[] a;
    private final double[] t;
    private final double ds;
    private final double length;

    private VelocityProfile(double[] s, double[] v, double[] a, double[] t, double ds, double length) {
        this.s = s;
        this.v = v;
        this.a = a;
        this.t = t;
        this.ds = ds;
        this.length = length;
    }

    public static final class Builder {
        private final Path path;
        private HeadingPlan heading;
        private FrictionEllipse ellipse = FrictionEllipse.typicalFtc();
        private double vStart = 0;
        private double vEnd = 0;
        private double resolution = 0.5;
        private double maxAngularAccel = 12.0;
        private double globalMaxVelocity = Double.MAX_VALUE;
        private List<RegionConstraint> regions;

        public Builder(Path path) {
            this.path = path;
            this.heading = new HeadingPlan.Tangent(path);
        }

        public Builder heading(HeadingPlan h) { this.heading = h; return this; }

        public Builder ellipse(FrictionEllipse e) { this.ellipse = e; return this; }

        public Builder startVelocity(double v) { this.vStart = Math.max(0, v); return this; }

        public Builder endVelocity(double v) { this.vEnd = Math.max(0, v); return this; }

        public Builder resolution(double inches) { this.resolution = Math.max(0.05, inches); return this; }

        public Builder maxAngularAccel(double v) { this.maxAngularAccel = v; return this; }

        public Builder maxVelocity(double v) { this.globalMaxVelocity = v; return this; }

        public Builder regions(List<RegionConstraint> r) { this.regions = r; return this; }

        public VelocityProfile build() {
            return VelocityProfile.build(path, heading, ellipse, vStart, vEnd, resolution,
                    maxAngularAccel, globalMaxVelocity, regions);
        }
    }

    public static Builder builder(Path path) { return new Builder(path); }

    private static VelocityProfile build(Path path, HeadingPlan heading, FrictionEllipse ellipse,
                                         double vStart, double vEnd, double resolution,
                                         double maxAngularAccel, double globalMax,
                                         List<RegionConstraint> regions) {
        double length = path.length();

        if (length < 1e-9) {
            return new VelocityProfile(new double[]{0, 0}, new double[]{0, 0},
                    new double[]{0, 0}, new double[]{0, 0}, 0, 0);
        }

        int n = MathUtil.gridNodes(length, resolution, MIN_NODES);
        double ds = length / (n - 1);

        double[] s = new double[n];
        double[] v = new double[n];
        double[] a = new double[n];
        double[] t = new double[n];
        double[] beta = new double[n];
        double[] k = new double[n];

        for (int i = 0; i < n; i++) {
            s[i] = ds * i;
            Path.State st = path.state(s[i]);

            double kMax = Math.abs(st.curvature);
            if (i < n - 1) {
                for (int sub = 1; sub < CURVATURE_SUBSAMPLES; sub++) {
                    double sSub = s[i] + ds * sub / (double) CURVATURE_SUBSAMPLES;
                    kMax = Math.max(kMax, Math.abs(path.curvature(sSub)));
                }
            }
            k[i] = kMax;

            double b = Utils.headingClip(st.tangentAngle() - heading.heading(s[i]));
            beta[i] = b;

            double cap = Math.min(globalMax, ellipse.maxSpeedForCurvature(k[i], b));
            cap = Math.min(cap, ellipse.maxSpeedFromWheels(b, heading.dHeadingDs(s[i])));

            double d2 = secondHeadingDeriv(heading, s[i], ds, length);
            if (Math.abs(d2) > 1e-9) {
                cap = Math.min(cap, Math.sqrt(maxAngularAccel / Math.abs(d2)));
            }

            if (regions != null) {
                Vector2 p = st.point;
                for (int r = 0; r < regions.size(); r++) {
                    cap = Math.min(cap, regions.get(r).cap(p));
                }
            }
            v[i] = cap;
        }

        v[0] = Math.min(v[0], vStart);
        for (int i = 0; i < n - 1; i++) {
            double aNear = ellipse.longitudinalAccelRange(v[i], k[i], beta[i])[1];
            double vPredict = Math.sqrt(Math.max(0, v[i] * v[i] + 2 * aNear * ds));
            double aFar = ellipse.longitudinalAccelRange(vPredict, k[i + 1], beta[i + 1])[1];
            double aUse = Math.min(aNear, aFar);
            double reachable = Math.sqrt(Math.max(0, v[i] * v[i] + 2 * aUse * ds));
            v[i + 1] = Math.min(v[i + 1], reachable);
        }

        v[n - 1] = Math.min(v[n - 1], vEnd);
        for (int i = n - 1; i > 0; i--) {
            double aNear = Math.abs(ellipse.longitudinalAccelRange(v[i], k[i], beta[i])[0]);
            double vPredict = Math.sqrt(Math.max(0, v[i] * v[i] + 2 * aNear * ds));
            double aFar = Math.abs(
                    ellipse.longitudinalAccelRange(vPredict, k[i - 1], beta[i - 1])[0]);
            double aUse = Math.min(aNear, aFar);
            double reachable = Math.sqrt(Math.max(0, v[i] * v[i] + 2 * aUse * ds));
            v[i - 1] = Math.min(v[i - 1], reachable);
        }

        for (int i = 0; i < n - 1; i++) {
            a[i] = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2 * ds);
        }
        a[n - 1] = a[n - 2];

        t[0] = 0;
        for (int i = 0; i < n - 1; i++) {
            double vm = 0.5 * (v[i] + v[i + 1]);
            t[i + 1] = t[i] + ds / Math.max(vm, 1e-3);
        }

        return new VelocityProfile(s, v, a, t, ds, length);
    }

    private static double secondHeadingDeriv(final HeadingPlan h, double s, double ds,
                                             double length) {
        return MathUtil.derivative(new MathUtil.Curve() {
            @Override public double at(double x) { return h.dHeadingDs(x); }
        }, s, Math.max(ds, 0.25), 0, length);
    }

    public double length() { return length; }

    public double duration() { return t[t.length - 1]; }

    public int size() { return s.length; }

    public double sAt(int i) { return s[i]; }

    public double velocity(double sQuery) {
        return MathUtil.interp(s, v, sQuery);
    }

    public double plannedAccel(double sQuery) {
        return MathUtil.interp(s, a, sQuery);
    }

    public double time(double sQuery) {
        return MathUtil.interp(s, t, sQuery);
    }

    public double arcLengthAtTime(double tQuery) {
        return MathUtil.interp(t, s, tQuery);
    }

    public double startVelocity() { return v[0]; }

    public double endVelocity() { return v[v.length - 1]; }

    public double maxVelocity() {
        double m = 0;
        for (double x : v) m = Math.max(m, x);
        return m;
    }

    public static double constantAccelTime(Path path, double accel, double maxVel,
                                           double vStart, double vEnd) {
        double d = path.length();
        double vPeak = Math.sqrt((2 * accel * d + vStart * vStart + vEnd * vEnd) / 2);
        if (vPeak <= maxVel) {
            return (vPeak - vStart) / accel + (vPeak - vEnd) / accel;
        }
        double dAccel = (maxVel * maxVel - vStart * vStart) / (2 * accel);
        double dBrake = (maxVel * maxVel - vEnd * vEnd) / (2 * accel);
        double dCruise = d - dAccel - dBrake;
        return (maxVel - vStart) / accel + dCruise / maxVel + (maxVel - vEnd) / accel;
    }

    @Override
    public String toString() {
        return String.format("VelocityProfile[%.1f in, %.2f s, vmax %.1f in/s, %d nodes]",
                length, duration(), maxVelocity(), s.length);
    }
}
