package org.firstinspires.ftc.teamcode.utils;

public class Vector2 {
    public double x;
    public double y;
    private double magcache;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        x = y = 0;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public double mag() {
        if (magcache == 0 && (x != 0 || y != 0)) {
            magcache = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        return magcache;
    }

    public void mul(double a) {
        x *= a;
        y *= a;
        magcache *= Math.abs(a);
    }

    public Vector2 plus(Vector2 a) { return new Vector2(x + a.x, y + a.y); }

    public Vector2 minus(Vector2 a) { return new Vector2(x - a.x, y - a.y); }

    public Vector2 times(double k) { return new Vector2(x * k, y * k); }

    public Vector2 div(double k) { return new Vector2(x / k, y / k); }

    public double dot(Vector2 a) { return x * a.x + y * a.y; }

    public double cross(Vector2 a) { return x * a.y - y * a.x; }

    public double mag2() { return x * x + y * y; }

    public Vector2 unit() {
        double m = Math.hypot(x, y);
        return m < 1e-12 ? new Vector2(0, 0) : new Vector2(x / m, y / m);
    }

    public Vector2 rotated(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Vector2(x * c - y * s, x * s + y * c);
    }

    public Vector2 leftNormal() { return new Vector2(-y, x); }

    public boolean isFinite() {
        return !Double.isNaN(x) && !Double.isNaN(y)
                && !Double.isInfinite(x) && !Double.isInfinite(y);
    }

    public static double distanceSquared(Vector2 a, Vector2 b) {
        double dx = a.x - b.x, dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    public static double dot(Vector2 a,Vector2 b) {
        return (a.x*b.x + a.y*b.y);
    }

    public void norm() {
        double mag = mag();

        if (mag == 0) {
            return;
        }

        x /= mag;
        y /= mag;
        magcache = 1;
    }

    public void add(Vector2 a) {
        x += a.x;
        y += a.y;
        magcache = 0;
    }

    public void subtract(Vector2 a) {
        x -= a.x;
        y -= a.y;
        magcache = 0;
    }

    public double theta() {
        return Math.atan2(y, x);
    }

    public String toString() {
        return String.format("(%f, %f)", x, y);
    }

    public static Vector2 subtract(Vector2 a, Vector2 b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    public static double distance(Vector2 v0, Vector2 v1) {
        return Math.sqrt(Math.pow(v0.x - v1.x, 2) + Math.pow(v0.y - v1.y, 2));
    }

    public void rotate(double rad) {
        double nx = x * Math.cos(rad) - y * Math.sin(rad);
        double ny = x * Math.sin(rad) + y * Math.cos(rad);

        x = nx;
        y = ny;
    }
}
