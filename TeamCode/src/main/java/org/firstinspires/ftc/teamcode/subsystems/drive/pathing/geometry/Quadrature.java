package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

final class Quadrature {
    private static final double[] X = {
            -0.906179845938664, -0.538469310105683, 0.0,
             0.538469310105683,  0.906179845938664
    };
    private static final double[] W = {
            0.236926885056189, 0.478628670499366, 0.568888888888889,
            0.478628670499366, 0.236926885056189
    };

    private Quadrature() {}

    static double integrateSpeed(BezierSegment seg, double t0, double t1, int n) {
        if (n < 1) n = 1;
        double h = (t1 - t0) / n;
        double total = 0;
        for (int k = 0; k < n; k++) {
            double a = t0 + k * h;
            double half = h * 0.5;
            double mid = a + half;
            double sum = 0;
            for (int i = 0; i < X.length; i++) {
                sum += W[i] * seg.speed(mid + half * X[i]);
            }
            total += sum * half;
        }
        return total;
    }
}
