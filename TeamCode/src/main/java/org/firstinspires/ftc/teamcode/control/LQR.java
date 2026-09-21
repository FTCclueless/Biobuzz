package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.utils.MatrixMath;

public final class LQR {
    private LQR() {}

    public static Matrix solve(Matrix A, Matrix B, Matrix Q, Matrix R) {
        return solve(A, B, Q, R, 2000, 1e-12);
    }

    public static Matrix solve(Matrix A, Matrix B, Matrix Q, Matrix R,
                               int maxIterations, double tolerance) {
        Matrix At = A.transpose();
        Matrix Bt = B.transpose();
        Matrix P = Q.copy();

        for (int i = 0; i < maxIterations; i++) {
            Matrix AtP = At.times(P);
            Matrix inner = R.plus(Bt.times(P).times(B)).inverse();
            Matrix next = AtP.times(A)
                    .minus(AtP.times(B).times(inner).times(Bt).times(P).times(A))
                    .plus(Q);
            double delta = next.minus(P).maxAbs();
            P = next;
            if (delta < tolerance) break;
        }

        if (!P.isFinite()) {
            throw new IllegalStateException(
                    "Riccati iteration diverged -- check that the model is controllable "
                            + "and that R is strictly positive");
        }

        Matrix inner = R.plus(Bt.times(P).times(B)).inverse();
        return inner.times(Bt).times(P).times(A);
    }

    public static Matrix[] discretize(Matrix Ac, Matrix Bc, double dt) {
        int n = Ac.rows;
        int m = Bc.cols;
        double[][] e = MatrixMath.exp(MatrixMath.blockAugmented(Ac.a, Bc.a, dt));
        return new Matrix[]{
                new Matrix(MatrixMath.submatrix(e, 0, 0, n, n)),
                new Matrix(MatrixMath.submatrix(e, 0, n, n, m))
        };
    }

    public static Matrix[] bryson(double[] maxError, double[] maxEffort) {
        double[] q = new double[maxError.length];
        for (int i = 0; i < maxError.length; i++) {
            q[i] = maxError[i] <= 0 ? 0 : 1.0 / (maxError[i] * maxError[i]);
        }
        double[] r = new double[maxEffort.length];
        for (int i = 0; i < maxEffort.length; i++) {
            if (maxEffort[i] <= 0) throw new IllegalArgumentException("Effort limits must be > 0");
            r[i] = 1.0 / (maxEffort[i] * maxEffort[i]);
        }
        return new Matrix[]{Matrix.diagonal(q), Matrix.diagonal(r)};
    }

    public static double closedLoopSpectralRadius(Matrix A, Matrix B, Matrix K) {
        Matrix cl = A.minus(B.times(K));
        int n = cl.rows;
        if (n == 1) return Math.abs(cl.a[0][0]);
        if (n == 2) {
            double tr = cl.a[0][0] + cl.a[1][1];
            double det = cl.a[0][0] * cl.a[1][1] - cl.a[0][1] * cl.a[1][0];
            double disc = tr * tr - 4 * det;
            if (disc >= 0) {
                double r1 = Math.abs((tr + Math.sqrt(disc)) / 2);
                double r2 = Math.abs((tr - Math.sqrt(disc)) / 2);
                return Math.max(r1, r2);
            }
            return Math.sqrt(det);
        }
        throw new UnsupportedOperationException("Only n <= 2 supported");
    }
}
