package org.firstinspires.ftc.teamcode.utils;

public final class MatrixMath {
    private MatrixMath() {}

    public static double[][] identity(int n) {
        double[][] out = new double[n][n];
        for (int i = 0; i < n; i++) out[i][i] = 1;
        return out;
    }

    public static double[][] diagonal(double... d) {
        double[][] out = new double[d.length][d.length];
        for (int i = 0; i < d.length; i++) out[i][i] = d[i];
        return out;
    }

    public static double[][] copy(double[][] a) {
        double[][] out = new double[a.length][];
        for (int i = 0; i < a.length; i++) out[i] = a[i].clone();
        return out;
    }

    public static double[][] multiply(double[][] a, double[][] b) {
        int rows = a.length, inner = b.length, cols = b[0].length;
        if (a[0].length != inner) {
            throw new IllegalArgumentException("Dimension mismatch: " + rows + "x"
                    + a[0].length + " * " + inner + "x" + cols);
        }
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int k = 0; k < inner; k++) {
                double aik = a[i][k];
                if (aik == 0) continue;
                double[] brow = b[k];
                double[] orow = out[i];
                for (int j = 0; j < cols; j++) orow[j] += aik * brow[j];
            }
        }
        return out;
    }

    public static double[][] gramian(double[][] a) {
        int rows = a.length, n = a[0].length;
        double[][] out = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = i; j < n; j++) {
                double acc = 0;
                for (int r = 0; r < rows; r++) acc += a[r][i] * a[r][j];
                out[i][j] = acc;
                out[j][i] = acc;
            }
        }
        return out;
    }

    public static double[][] transpose(double[][] a) {
        int rows = a.length, cols = a[0].length;
        double[][] out = new double[cols][rows];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) out[j][i] = a[i][j];
        return out;
    }

    public static double[][] scale(double[][] a, double k) {
        int rows = a.length, cols = a[0].length;
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) out[i][j] = a[i][j] * k;
        return out;
    }

    public static double[][] add(double[][] a, double[][] b) {
        int rows = a.length, cols = a[0].length;
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) out[i][j] = a[i][j] + b[i][j];
        return out;
    }

    public static double[][] subtract(double[][] a, double[][] b) {
        int rows = a.length, cols = a[0].length;
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) out[i][j] = a[i][j] - b[i][j];
        return out;
    }

    public static double[] matVec(double[][] a, double[] x) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            double[] row = a[i];
            double acc = 0;
            for (int j = 0; j < x.length; j++) acc += row[j] * x[j];
            out[i] = acc;
        }
        return out;
    }

    public static double[] transposeMatVec(double[][] a, double[] x) {
        int cols = a[0].length;
        double[] out = new double[cols];
        for (int j = 0; j < cols; j++) {
            double acc = 0;
            for (int r = 0; r < a.length; r++) acc += a[r][j] * x[r];
            out[j] = acc;
        }
        return out;
    }

    public static double rowDot(double[][] a, int row, double[] x) {
        return rowDot(a, row, x, 0);
    }

    public static double rowDot(double[][] a, int row, double[] x, double initial) {
        double[] r = a[row];
        double acc = initial;
        for (int j = 0; j < x.length; j++) acc += r[j] * x[j];
        return acc;
    }

    public static double[] matVecAdd(double[][] a, double[] x, double[] base) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++) out[i] = rowDot(a, i, x, base[i]);
        return out;
    }

    public static double[][] rowColumnScaled(double[][] a, double[] rowScale, double[] colScale) {
        int rows = a.length, cols = a[0].length;
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            double[] row = a[i];
            double[] orow = out[i];
            double rs = rowScale[i];
            for (int k = 0; k < cols; k++) orow[k] = rs * row[k] * colScale[k];
        }
        return out;
    }

    public static double quadratic(double[][] h, double[] x) {
        double acc = 0;
        for (int i = 0; i < x.length; i++) acc += x[i] * rowDot(h, i, x);
        return acc;
    }

    public static double dot(double[] a, double[] b) {
        double acc = 0;
        for (int i = 0; i < a.length; i++) acc += a[i] * b[i];
        return acc;
    }

    public static double[][] inverse(double[][] a) {
        int n = a.length;
        if (a[0].length != n) throw new IllegalArgumentException("Only square matrices invert");
        double[][] m = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, m[i], 0, n);
            m[i][n + i] = 1;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                throw new IllegalArgumentException("Matrix is singular");
            }
            double[] tmp = m[col]; m[col] = m[pivot]; m[pivot] = tmp;

            double d = m[col][col];
            for (int j = 0; j < 2 * n; j++) m[col][j] /= d;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double f = m[r][col];
                if (f == 0) continue;
                for (int j = 0; j < 2 * n; j++) m[r][j] -= f * m[col][j];
            }
        }
        double[][] out = new double[n][n];
        for (int i = 0; i < n; i++) System.arraycopy(m[i], n, out[i], 0, n);
        return out;
    }

    public static double[][] exp(double[][] a) {
        int n = a.length;
        if (a[0].length != n) throw new IllegalArgumentException("exp() needs a square matrix");
        double norm = maxAbs(a);
        int squarings = 0;
        double scale = 1.0;
        while (norm * scale > 0.5) {
            scale *= 0.5;
            squarings++;
        }
        double[][] scaled = scale(a, scale);

        double[][] result = identity(n);
        double[][] term = identity(n);
        for (int k = 1; k <= 20; k++) {
            term = scale(multiply(term, scaled), 1.0 / k);
            result = add(result, term);
            if (maxAbs(term) < 1e-18) break;
        }
        for (int i = 0; i < squarings; i++) {
            result = multiply(result, result);
        }
        return result;
    }

    public static double[][] submatrix(double[][] a, int r0, int c0, int rows, int cols) {
        double[][] out = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(a[r0 + i], c0, out[i], 0, cols);
        }
        return out;
    }

    public static double[][] blockAugmented(double[][] a, double[][] b, double k) {
        int n = a.length;
        int m = b[0].length;
        double[][] out = new double[n + m][n + m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) out[i][j] = a[i][j] * k;
            for (int j = 0; j < m; j++) out[i][n + j] = b[i][j] * k;
        }
        return out;
    }

    public static double maxAbs(double[][] a) {
        double m = 0;
        for (double[] row : a)
            for (double v : row) m = Math.max(m, Math.abs(v));
        return m;
    }

    public static boolean isFinite(double[][] a) {
        for (double[] row : a)
            for (double v : row) {
                if (Double.isNaN(v) || Double.isInfinite(v)) return false;
            }
        return true;
    }

    public static void tridiagonal(double[] sub, double[] diag, double[] sup,
                                   double[] rhs, double[] out) {
        int n = diag.length;
        double[] cp = new double[n];
        double[] dp = new double[n];
        cp[0] = sup[0] / diag[0];
        dp[0] = rhs[0] / diag[0];
        for (int i = 1; i < n; i++) {
            double denom = diag[i] - sub[i] * cp[i - 1];
            cp[i] = sup[i] / denom;
            dp[i] = (rhs[i] - sub[i] * dp[i - 1]) / denom;
        }
        out[n - 1] = dp[n - 1];
        for (int i = n - 2; i >= 0; i--) {
            out[i] = dp[i] - cp[i] * out[i + 1];
        }
    }

    public static double[][] tridiagonalSolveMatrix(double[] sub, double[] diag, double[] sup,
                                                    double[][] b) {
        int n = diag.length;
        double[][] out = new double[n][n];
        double[] col = new double[n];
        double[] sol = new double[n];
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) col[i] = b[i][k];
            tridiagonal(sub, diag, sup, col, sol);
            for (int i = 0; i < n; i++) out[i][k] = sol[i];
        }
        return out;
    }
}
