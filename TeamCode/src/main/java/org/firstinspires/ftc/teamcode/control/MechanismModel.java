package org.firstinspires.ftc.teamcode.control;

public final class MechanismModel {
    public enum Type {
        POSITION,
        VELOCITY
    }

    public final Type type;
    public final double kV;
    public final double kA;
    public final double kS;
    public final double kG;
    public final double nominalVoltage;

    public MechanismModel(Type type, double kV, double kA, double kS, double kG,
                          double nominalVoltage) {
        if (kV <= 0) throw new IllegalArgumentException("kV must be > 0");
        if (kA <= 0) throw new IllegalArgumentException("kA must be > 0 (an ideal massless "
                + "mechanism has no dynamics to control)");
        this.type = type;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kG = kG;
        this.nominalVoltage = nominalVoltage;
    }

    public static MechanismModel horizontalSlide(double kV, double kA, double kS) {
        return new MechanismModel(Type.POSITION, kV, kA, kS, 0.0, 12.0);
    }

    public static MechanismModel verticalSlide(double kV, double kA, double kS, double kG) {
        return new MechanismModel(Type.POSITION, kV, kA, kS, kG, 12.0);
    }

    public static MechanismModel flywheel(double kV, double kA, double kS) {
        return new MechanismModel(Type.VELOCITY, kV, kA, kS, 0.0, 12.0);
    }

    public int stateCount() { return type == Type.POSITION ? 2 : 1; }

    public Matrix continuousA() {
        if (type == Type.POSITION) {
            return new Matrix(new double[][]{
                    {0, 1},
                    {0, -kV / kA}
            });
        }
        return new Matrix(new double[][]{{-kV / kA}});
    }

    public Matrix continuousB() {
        if (type == Type.POSITION) {
            return new Matrix(new double[][]{{0}, {1.0 / kA}});
        }
        return new Matrix(new double[][]{{1.0 / kA}});
    }

    public double timeConstant() { return kA / kV; }

    public double freeSpeed(double volts) { return (volts - kS - kG) / kV; }

    public double feedforward(double velocity, double acceleration) {
        double v = kV * velocity + kA * acceleration + kG;
        if (Math.abs(velocity) > 1e-6) {
            v += Math.copySign(kS, velocity);
        } else if (Math.abs(acceleration) > 1e-6) {
            v += Math.copySign(kS, acceleration);
        }
        return v;
    }

    @Override
    public String toString() {
        return String.format("MechanismModel[%s kV=%.4f kA=%.4f kS=%.3f kG=%.3f tau=%.3fs]",
                type, kV, kA, kS, kG, timeConstant());
    }
}
