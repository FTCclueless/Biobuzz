package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

import org.firstinspires.ftc.teamcode.utils.Vector2;

public final class BezierSegment {
    private final Vector2[] ctrl;
    private final Vector2[] d1;
    private final Vector2[] d2;

    public BezierSegment(Vector2... controlPoints) {
        if (controlPoints.length < 2) {
            throw new IllegalArgumentException("A Bezier needs at least 2 control points");
        }
        this.ctrl = controlPoints.clone();
        this.d1 = hodograph(this.ctrl);
        this.d2 = hodograph(this.d1);
    }

    private static Vector2[] hodograph(Vector2[] p) {
        int n = p.length - 1;
        if (n < 1) return new Vector2[] { new Vector2(0, 0) };
        Vector2[] out = new Vector2[n];
        for (int i = 0; i < n; i++) {
            out[i] = p[i + 1].minus(p[i]).times(n);
        }
        return out;
    }

    public int degree() { return ctrl.length - 1; }

    public Vector2 controlPoint(int i) { return ctrl[i]; }

    public Vector2 start() { return ctrl[0]; }

    public Vector2 end() { return ctrl[ctrl.length - 1]; }

    public Vector2 point(double t) { return deCasteljau(ctrl, t); }

    public Vector2 deriv(double t) { return deCasteljau(d1, t); }

    public Vector2 deriv2(double t) { return deCasteljau(d2, t); }

    public double speed(double t) {
        int n = d1.length;
        switch (n) {
            case 1:
                return d1[0].mag();
            case 2: {
                Vector2 p0 = d1[0], p1 = d1[1];
                double x = p0.x + (p1.x - p0.x) * t;
                double y = p0.y + (p1.y - p0.y) * t;
                return Math.sqrt(x * x + y * y);
            }
            case 3: {
                double u = 1 - t;
                double b0 = u * u, b1 = 2 * u * t, b2 = t * t;
                Vector2 p0 = d1[0], p1 = d1[1], p2 = d1[2];
                double x = b0 * p0.x + b1 * p1.x + b2 * p2.x;
                double y = b0 * p0.y + b1 * p1.y + b2 * p2.y;
                return Math.sqrt(x * x + y * y);
            }
            case 4: {
                double u = 1 - t;
                double uu = u * u, tt = t * t;
                double b0 = uu * u, b1 = 3 * uu * t, b2 = 3 * u * tt, b3 = tt * t;
                Vector2 p0 = d1[0], p1 = d1[1], p2 = d1[2], p3 = d1[3];
                double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
                double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
                return Math.sqrt(x * x + y * y);
            }
            default:
                return deriv(t).mag();
        }
    }

    public double curvature(double t) {
        Vector2 d = deriv(t);
        Vector2 dd = deriv2(t);
        double speed = d.mag();
        if (speed < 1e-9) return 0;
        return d.cross(dd) / (speed * speed * speed);
    }

    private static void evalInto(Vector2[] pts, double t, double[] out, int off) {
        switch (pts.length) {
            case 1:
                out[off] = pts[0].x;
                out[off + 1] = pts[0].y;
                return;
            case 2: {
                Vector2 p0 = pts[0], p1 = pts[1];
                out[off] = p0.x + (p1.x - p0.x) * t;
                out[off + 1] = p0.y + (p1.y - p0.y) * t;
                return;
            }
            case 3: {
                double u = 1 - t;
                double b0 = u * u, b1 = 2 * u * t, b2 = t * t;
                Vector2 p0 = pts[0], p1 = pts[1], p2 = pts[2];
                out[off] = b0 * p0.x + b1 * p1.x + b2 * p2.x;
                out[off + 1] = b0 * p0.y + b1 * p1.y + b2 * p2.y;
                return;
            }
            case 4: {
                double u = 1 - t;
                double uu = u * u, tt = t * t;
                double b0 = uu * u, b1 = 3 * uu * t, b2 = 3 * u * tt, b3 = tt * t;
                Vector2 p0 = pts[0], p1 = pts[1], p2 = pts[2], p3 = pts[3];
                out[off] = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
                out[off + 1] = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
                return;
            }
            default: {
                Vector2 v = deCasteljau(pts, t);
                out[off] = v.x;
                out[off + 1] = v.y;
            }
        }
    }

    public void pointInto(double t, double[] out, int off) { evalInto(ctrl, t, out, off); }

    public void derivInto(double t, double[] out, int off) { evalInto(d1, t, out, off); }

    public void deriv2Into(double t, double[] out, int off) { evalInto(d2, t, out, off); }

    public double curvature(double t, double[] scratch, int off) {
        derivInto(t, scratch, off);
        deriv2Into(t, scratch, off + 2);
        double dx = scratch[off], dy = scratch[off + 1];
        double ddx = scratch[off + 2], ddy = scratch[off + 3];
        double sp2 = dx * dx + dy * dy;
        if (sp2 < 1e-18) return 0;
        return (dx * ddy - dy * ddx) / (sp2 * Math.sqrt(sp2));
    }

    private static Vector2 deCasteljau(Vector2[] pts, double t) {
        switch (pts.length) {
            case 1:
                return pts[0];
            case 2: {
                Vector2 p0 = pts[0], p1 = pts[1];
                return new Vector2(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t);
            }
            case 3: {
                double u = 1 - t;
                double b0 = u * u, b1 = 2 * u * t, b2 = t * t;
                Vector2 p0 = pts[0], p1 = pts[1], p2 = pts[2];
                return new Vector2(b0 * p0.x + b1 * p1.x + b2 * p2.x,
                                b0 * p0.y + b1 * p1.y + b2 * p2.y);
            }
            case 4: {
                double u = 1 - t;
                double uu = u * u, tt = t * t;
                double b0 = uu * u, b1 = 3 * uu * t, b2 = 3 * u * tt, b3 = tt * t;
                Vector2 p0 = pts[0], p1 = pts[1], p2 = pts[2], p3 = pts[3];
                return new Vector2(b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
                                b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y);
            }
            default:
                break;
        }
        int n = pts.length;
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = pts[i].x;
            ys[i] = pts[i].y;
        }
        for (int k = 1; k < n; k++) {
            for (int i = 0; i < n - k; i++) {
                xs[i] = xs[i] + (xs[i + 1] - xs[i]) * t;
                ys[i] = ys[i] + (ys[i + 1] - ys[i]) * t;
            }
        }
        return new Vector2(xs[0], ys[0]);
    }

    public double length() { return length(0, 1, 16); }

    public double length(double t0, double t1, int subdivisions) {
        return Quadrature.integrateSpeed(this, t0, t1, subdivisions);
    }

    public static BezierSegment cubicFromHermite(Vector2 p0, Vector2 v0, Vector2 p1, Vector2 v1) {
        return new BezierSegment(
                p0,
                p0.plus(v0.div(3.0)),
                p1.minus(v1.div(3.0)),
                p1);
    }

    public static BezierSegment quinticFromHermite(Vector2 p0, Vector2 v0, Vector2 a0,
                                                   Vector2 p1, Vector2 v1, Vector2 a1) {
        Vector2 c0 = p0;
        Vector2 c1 = p0.plus(v0.div(5.0));
        Vector2 c2 = p0.plus(v0.times(2.0 / 5.0)).plus(a0.div(20.0));
        Vector2 c3 = p1.minus(v1.times(2.0 / 5.0)).plus(a1.div(20.0));
        Vector2 c4 = p1.minus(v1.div(5.0));
        Vector2 c5 = p1;
        return new BezierSegment(c0, c1, c2, c3, c4, c5);
    }

    public static BezierSegment line(Vector2 a, Vector2 b) {
        return new BezierSegment(a, b);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Bezier[deg=").append(degree()).append("] ");
        for (Vector2 p : ctrl) sb.append(p).append(' ');
        return sb.toString().trim();
    }
}
