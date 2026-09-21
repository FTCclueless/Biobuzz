package org.firstinspires.ftc.teamcode.control;

public final class StatePredictor {
    private static final double FEEDTHROUGH_MARGIN = 0.5;

    private final boolean position;
    private final double kG;
    private final double a00, a01, a10, a11;
    private final double b0, b1;

    private double trust = 1.0;
    private double predictedPosition;
    private double predictedVelocity;

    private StatePredictor(MechanismModel model, Matrix ad, Matrix bd) {
        this.position = model.type == MechanismModel.Type.POSITION;
        this.kG = model.kG;
        if (position) {
            this.a00 = ad.get(0, 0);
            this.a01 = ad.get(0, 1);
            this.a10 = ad.get(1, 0);
            this.a11 = ad.get(1, 1);
            this.b0 = bd.get(0, 0);
            this.b1 = bd.get(1, 0);
        } else {
            this.a00 = ad.get(0, 0);
            this.a01 = 0;
            this.a10 = 0;
            this.a11 = 0;
            this.b0 = bd.get(0, 0);
            this.b1 = 0;
        }
    }

    public static StatePredictor create(MechanismModel model, double latency) {
        if (latency <= 1e-6) return null;
        Matrix[] d = LQR.discretize(model.continuousA(), model.continuousB(), latency);
        return new StatePredictor(model, d[0], d[1]);
    }

    public void updateTrust(Matrix k) {
        trust = 1.0;
        if (k == null) return;
        double feedthrough = position
                ? Math.abs(k.get(0, 0) * b0 + k.get(0, 1) * b1)
                : Math.abs(k.get(0, 0) * b0);
        if (feedthrough > FEEDTHROUGH_MARGIN) {
            trust = FEEDTHROUGH_MARGIN / feedthrough;
        }
    }

    public double trust() { return trust; }

    public void update(double measuredPosition, double measuredVelocity, double lastOutput) {
        if (position) {
            double u = lastOutput - kG;
            predictedPosition = a00 * measuredPosition + a01 * measuredVelocity + b0 * u;
            predictedVelocity = a10 * measuredPosition + a11 * measuredVelocity + b1 * u;
        } else {
            predictedPosition = measuredPosition;
            predictedVelocity = a00 * measuredVelocity + b0 * lastOutput;
        }
        if (trust < 1.0) {
            predictedPosition = measuredPosition + (predictedPosition - measuredPosition) * trust;
            predictedVelocity = measuredVelocity + (predictedVelocity - measuredVelocity) * trust;
        }
    }

    public double position() { return predictedPosition; }

    public double velocity() { return predictedVelocity; }
}
