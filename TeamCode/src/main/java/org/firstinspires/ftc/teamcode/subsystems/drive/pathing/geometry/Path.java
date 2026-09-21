package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class Path {
    private static final double NODES_PER_INCH = 4.0;
    private static final int MIN_NODES = 40;
    private static final int MAX_NODES = 400;

    private final BezierSegment[] segments;
    private final double[][] segS;
    private final double[] segStartS;
    private final double length;

    public Path(List<BezierSegment> segs) {
        if (segs.isEmpty()) throw new IllegalArgumentException("A path needs at least one segment");
        this.segments = segs.toArray(new BezierSegment[0]);
        this.segS = new double[segments.length][];
        this.segStartS = new double[segments.length + 1];

        double acc = 0;
        for (int i = 0; i < segments.length; i++) {
            segStartS[i] = acc;
            segS[i] = buildTable(segments[i]);
            acc += segS[i][segS[i].length - 1];
        }
        segStartS[segments.length] = acc;
        this.length = acc;
    }

    public Path(BezierSegment... segs) {
        this(Arrays.asList(segs));
    }

    public static Path ofLine(Vector2 a, Vector2 b) {
        return new Path(Collections.singletonList(BezierSegment.line(a, b)));
    }

    private static double[] buildTable(BezierSegment seg) {
        double poly = 0;
        for (int i = 0; i + 1 <= seg.degree(); i++) {
            poly += Vector2.distance(seg.controlPoint(i), seg.controlPoint(i + 1));
        }
        int m = Utils.minMaxClip((int) Math.ceil(poly * NODES_PER_INCH), MIN_NODES, MAX_NODES);
        double[] table = new double[m + 1];
        double acc = 0;
        table[0] = 0;
        for (int j = 0; j < m; j++) {
            double t0 = (double) j / m;
            double t1 = (double) (j + 1) / m;
            acc += seg.length(t0, t1, 1);
            table[j + 1] = acc;
        }
        return table;
    }

    public double length() { return length; }

    public int segmentCount() { return segments.length; }

    public BezierSegment segment(int i) { return segments[i]; }

    public Vector2 startPoint() { return segments[0].start(); }

    public Vector2 endPoint() { return segments[segments.length - 1].end(); }

    public static final class Loc {
        public int seg;
        public double t;
    }

    public static final class Sample {
        public double s;
        public double px, py;
        public double tx, ty;
        public double curvature;
        public int seg;
        public double t;
        final double[] scratch = new double[4];

        public double nx() { return -ty; }

        public double ny() { return tx; }

        public double tangentAngle() { return Math.atan2(ty, tx); }
    }

    public void locate(double s, Loc out) {
        double sc = Utils.minMaxClip(s, 0, length);
        int i = MathUtil.lowerBound(segStartS, sc);
        if (i >= segments.length) i = segments.length - 1;
        double[] tbl = segS[i];
        int m = tbl.length - 1;
        double local = Utils.minMaxClip(sc - segStartS[i], 0, tbl[m]);

        int j = MathUtil.lowerBound(tbl, local);
        double t0 = (double) j / m;
        double t1 = (double) (j + 1) / m;
        double span = tbl[j + 1] - tbl[j];
        double t = span < 1e-12 ? t0 : t0 + (t1 - t0) * (local - tbl[j]) / span;

        BezierSegment seg = segments[i];
        double sAtT = tbl[j] + seg.length(t0, t, 1);
        double speed = seg.speed(t);
        if (speed > 1e-9) {
            t += (local - sAtT) / speed;
            t = Utils.minMaxClip(t, t0, t1);
        }

        out.seg = i;
        out.t = t;
    }

    private void locateFast(double s, Sample out) {
        double sc = Utils.minMaxClip(s, 0, length);
        int i = MathUtil.lowerBound(segStartS, sc);
        if (i >= segments.length) i = segments.length - 1;
        double[] tbl = segS[i];
        int m = tbl.length - 1;
        double local = Utils.minMaxClip(sc - segStartS[i], 0, tbl[m]);
        int j = MathUtil.lowerBound(tbl, local);
        double span = tbl[j + 1] - tbl[j];
        double t = span < 1e-12
                ? (double) j / m
                : ((double) j + (local - tbl[j]) / span) / m;
        out.seg = i;
        out.t = t;
    }

    public void sample(double s, Sample out) {
        double sc = Utils.minMaxClip(s, 0, length);
        int i = MathUtil.lowerBound(segStartS, sc);
        if (i >= segments.length) i = segments.length - 1;
        double[] tbl = segS[i];
        int m = tbl.length - 1;
        double local = Utils.minMaxClip(sc - segStartS[i], 0, tbl[m]);

        int j = MathUtil.lowerBound(tbl, local);
        double t0 = (double) j / m;
        double t1 = (double) (j + 1) / m;
        double span = tbl[j + 1] - tbl[j];
        double t = span < 1e-12 ? t0 : t0 + (t1 - t0) * (local - tbl[j]) / span;

        BezierSegment seg = segments[i];
        double sAtT = tbl[j] + seg.length(t0, t, 1);
        double sp = seg.speed(t);
        if (sp > 1e-9) {
            t += (local - sAtT) / sp;
            t = Utils.minMaxClip(t, t0, t1);
        }

        out.s = sc;
        out.seg = i;
        out.t = t;
        seg.pointInto(t, out.scratch, 0);
        out.px = out.scratch[0];
        out.py = out.scratch[1];
        seg.derivInto(t, out.scratch, 0);
        double dx = out.scratch[0], dy = out.scratch[1];
        double n = Math.sqrt(dx * dx + dy * dy);
        if (n < 1e-12) {
            out.tx = 1;
            out.ty = 0;
        } else {
            out.tx = dx / n;
            out.ty = dy / n;
        }
        out.curvature = seg.curvature(t, out.scratch, 0);
    }

    public double arcLengthAt(int seg, double t) {
        double[] tbl = segS[seg];
        int m = tbl.length - 1;
        double tc = Utils.minMaxClip(t, 0, 1);
        int j = Utils.minMaxClip((int) (tc * m), 0, m - 1);
        return segStartS[seg] + tbl[j] + segments[seg].length((double) j / m, tc, 2);
    }

    public Vector2 point(double s) {
        Loc l = new Loc();
        locate(s, l);
        return segments[l.seg].point(l.t);
    }

    public Vector2 tangent(double s) {
        Loc l = new Loc();
        locate(s, l);
        return segments[l.seg].deriv(l.t).unit();
    }

    public Vector2 normal(double s) {
        return tangent(s).leftNormal();
    }

    public double curvature(double s) {
        Loc l = new Loc();
        locate(s, l);
        return segments[l.seg].curvature(l.t);
    }

    private final MathUtil.Curve curvatureCurve = new MathUtil.Curve() {
        @Override public double at(double x) { return curvature(x); }
    };

    public double dCurvatureDs(double s) {
        return MathUtil.derivative(curvatureCurve, s, 0.25, 0, length);
    }

    public static final class State {
        public final double s;
        public final Vector2 point;
        public final Vector2 tangent;
        public final Vector2 normal;
        public final double curvature;

        public State(double s, Vector2 point, Vector2 tangent, Vector2 normal, double curvature) {
            this.s = s;
            this.point = point;
            this.tangent = tangent;
            this.normal = normal;
            this.curvature = curvature;
        }

        public double tangentAngle() { return tangent.theta(); }
    }

    public State state(double s) {
        Loc l = new Loc();
        locate(s, l);
        BezierSegment seg = segments[l.seg];
        Vector2 p = seg.point(l.t);
        Vector2 d = seg.deriv(l.t);
        Vector2 t = d.unit();
        return new State(Utils.minMaxClip(s, 0, length), p, t, t.leftNormal(), seg.curvature(l.t));
    }

    public double project(Vector2 q, double sGuess, double window) {
        return project(q.x, q.y, sGuess, window, null);
    }

    public double project(double qx, double qy, double sGuess, double window, Sample scratch) {
        double lo = Utils.minMaxClip(sGuess - window, 0, length);
        double hi = Utils.minMaxClip(sGuess + window, 0, length);
        if (hi - lo < 1e-9) return lo;

        Sample sm = scratch != null ? scratch : new Sample();

        final int samples = 24;
        double best = lo;
        double bestD2 = Double.MAX_VALUE;
        double invSamples = (hi - lo) / samples;
        for (int i = 0; i <= samples; i++) {
            double s = lo + invSamples * i;
            locateFast(s, sm);
            segments[sm.seg].pointInto(sm.t, sm.scratch, 0);
            double dx = sm.scratch[0] - qx;
            double dy = sm.scratch[1] - qy;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                best = s;
            }
        }
        sample(best, sm);
        double bdx = sm.px - qx, bdy = sm.py - qy;
        bestD2 = bdx * bdx + bdy * bdy;

        double s = best;
        double span = hi - lo;
        for (int it = 0; it < 5; it++) {
            sample(s, sm);
            double rx = sm.px - qx, ry = sm.py - qy;
            double f = rx * sm.tx + ry * sm.ty;
            double fp = 1.0 + (rx * sm.nx() + ry * sm.ny()) * sm.curvature;
            if (Math.abs(fp) < 1e-6) break;
            double step = f / fp;
            step = Utils.minMaxClip(step, -span, span);
            double next = Utils.minMaxClip(s - step, lo, hi);
            if (Math.abs(next - s) < 1e-6) { s = next; break; }
            s = next;
        }
        sample(s, sm);
        double fx = sm.px - qx, fy = sm.py - qy;
        return (fx * fx + fy * fy) <= bestD2 ? s : best;
    }

    public double peakCurvature() {
        int samples = Math.max(200, (int) (length * 8));
        double peak = 0;
        for (int i = 0; i <= samples; i++) {
            peak = Math.max(peak, Math.abs(curvature(length * i / samples)));
        }
        return peak;
    }

    public boolean hasCusp(double radiusInches) {
        return peakCurvature() > 1.0 / Math.max(radiusInches, 1e-6);
    }

    public boolean hasCusp() { return hasCusp(1.0); }

    public List<Vector2> sampleUniform(int count) {
        List<Vector2> out = new ArrayList<Vector2>(Math.max(count, 0));
        if (count <= 0) return out;
        if (count == 1) {
            out.add(startPoint());
            return out;
        }
        for (int i = 0; i < count; i++) {
            out.add(point(length * i / (count - 1.0)));
        }
        return out;
    }

    @Override
    public String toString() {
        return String.format("Path[%d segments, %.2f in]", segments.length, length);
    }
}
