package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.utils.MatrixMath;

public final class Matrix {
    public static long allocations = 0;

    public final int rows;
    public final int cols;
    public final double[][] a;

    public Matrix(int rows, int cols) {
        allocations++;
        this.rows = rows;
        this.cols = cols;
        this.a = new double[rows][cols];
    }

    public Matrix(double[][] values) {
        this(MatrixMath.copy(values), true);
    }

    private Matrix(double[][] adopted, boolean adopt) {
        allocations++;
        this.rows = adopted.length;
        this.cols = adopted[0].length;
        this.a = adopted;
    }

    private static Matrix wrap(double[][] values) { return new Matrix(values, true); }

    public static Matrix identity(int n) { return wrap(MatrixMath.identity(n)); }

    public static Matrix diagonal(double... d) { return wrap(MatrixMath.diagonal(d)); }

    public static Matrix column(double... v) {
        Matrix m = new Matrix(v.length, 1);
        for (int i = 0; i < v.length; i++) m.a[i][0] = v[i];
        return m;
    }

    public double get(int r, int c) { return a[r][c]; }

    public void set(int r, int c, double v) { a[r][c] = v; }

    public Matrix times(Matrix o) { return wrap(MatrixMath.multiply(a, o.a)); }

    public Matrix times(double k) { return wrap(MatrixMath.scale(a, k)); }

    public Matrix plus(Matrix o) { return wrap(MatrixMath.add(a, o.a)); }

    public Matrix minus(Matrix o) { return wrap(MatrixMath.subtract(a, o.a)); }

    public Matrix transpose() { return wrap(MatrixMath.transpose(a)); }

    public Matrix inverse() { return wrap(MatrixMath.inverse(a)); }

    public Matrix exp() { return wrap(MatrixMath.exp(a)); }

    public Matrix copy() { return wrap(MatrixMath.copy(a)); }

    public double maxAbs() { return MatrixMath.maxAbs(a); }

    public boolean isFinite() { return MatrixMath.isFinite(a); }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < rows; i++) {
            sb.append('[');
            for (int j = 0; j < cols; j++) {
                sb.append(String.format("%10.5f", a[i][j]));
                if (j < cols - 1) sb.append(' ');
            }
            sb.append("]\n");
        }
        return sb.toString();
    }
}
