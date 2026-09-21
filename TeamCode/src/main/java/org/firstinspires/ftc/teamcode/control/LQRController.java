package org.firstinspires.ftc.teamcode.control;

public final class LQRController {
    private final MechanismModel model;
    private final double dt;
    private final Matrix A;
    private final Matrix B;

    private Matrix K;
    private StatePredictor predictor;
    private double minOutput = -12.0;
    private double maxOutput = 12.0;
    private double positionTolerance = 0.5;
    private double velocityTolerance = 2.0;
    private boolean voltageCompensation = true;

    private MotionProfile profile;
    private double profileTime = 0;
    private double goalPosition = 0;
    private double goalVelocity = 0;
    private double refPosition = 0;
    private double refVelocity = 0;
    private double refAccel = 0;

    private double lastFeedforward = 0;
    private double lastFeedback = 0;
    private double lastOutput = 0;
    private boolean lastSaturated = false;

    public LQRController(MechanismModel model, double dt) {
        if (dt <= 0) throw new IllegalArgumentException("dt must be > 0");
        this.model = model;
        this.dt = dt;
        Matrix[] discrete = LQR.discretize(model.continuousA(), model.continuousB(), dt);
        this.A = discrete[0];
        this.B = discrete[1];
        if (model.type == MechanismModel.Type.POSITION) {
            tolerances(new double[]{0.5, 5.0}, 10.0);
        } else {
            tolerances(new double[]{10.0}, 10.0);
        }
    }

    public LQRController tolerances(double[] maxError, double maxEffort) {
        if (maxError.length != model.stateCount()) {
            throw new IllegalArgumentException("Expected " + model.stateCount()
                    + " state tolerances for a " + model.type + " mechanism, got "
                    + maxError.length);
        }
        Matrix[] qr = LQR.bryson(maxError, new double[]{maxEffort});
        this.K = LQR.solve(A, B, qr[0], qr[1]);
        refreshPredictorTrust();
        return this;
    }

    public LQRController weights(Matrix Q, Matrix R) {
        this.K = LQR.solve(A, B, Q, R);
        refreshPredictorTrust();
        return this;
    }

    public LQRController outputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
        return this;
    }

    public LQRController latencyCompensation(double seconds) {
        this.predictor = StatePredictor.create(model, Math.max(0, seconds));
        refreshPredictorTrust();
        return this;
    }

    private void refreshPredictorTrust() {
        if (predictor != null) predictor.updateTrust(K);
    }

    public double latencyTrust() { return predictor == null ? 1.0 : predictor.trust(); }

    public LQRController voltageCompensation(boolean on) {
        this.voltageCompensation = on;
        return this;
    }

    public LQRController atGoalTolerance(double position, double velocity) {
        this.positionTolerance = position;
        this.velocityTolerance = velocity;
        return this;
    }

    public void setGoal(double position, double maxVelocity, double maxAccel) {
        requirePosition();
        this.profile = new MotionProfile(refPosition, position, maxVelocity, maxAccel);
        this.profileTime = 0;
        this.goalPosition = position;
        this.goalVelocity = 0;
    }

    public void setGoalFrom(double currentPosition, double position,
                            double maxVelocity, double maxAccel) {
        requirePosition();
        this.refPosition = currentPosition;
        setGoal(position, maxVelocity, maxAccel);
    }

    public void setPositionGoal(double position) {
        requirePosition();
        this.profile = null;
        this.goalPosition = position;
        this.goalVelocity = 0;
        this.refPosition = position;
        this.refVelocity = 0;
        this.refAccel = 0;
    }

    public void setVelocityGoal(double velocity) {
        this.profile = null;
        this.goalVelocity = velocity;
        this.refVelocity = velocity;
        this.refAccel = 0;
    }

    private void requirePosition() {
        if (model.type != MechanismModel.Type.POSITION) {
            throw new IllegalStateException(
                    "Position goals are meaningless on a VELOCITY mechanism. A flywheel's "
                            + "position is not a thing you want to control.");
        }
    }

    public double calculate(double position, double velocity, double batteryVolts) {
        if (profile != null) {
            profileTime += dt;
            MotionProfile.State st = profile.get(profileTime);
            refPosition = st.position;
            refVelocity = st.velocity;
            refAccel = st.acceleration;
        } else {
            refVelocity = goalVelocity;
            refAccel = 0;
        }

        double predPos = position;
        double predVel = velocity;
        if (predictor != null) {
            predictor.update(position, velocity, lastOutput);
            predPos = predictor.position();
            predVel = predictor.velocity();
        }

        double ff = model.feedforward(refVelocity, refAccel);

        double fb;
        if (model.type == MechanismModel.Type.POSITION) {
            fb = K.get(0, 0) * (refPosition - predPos) + K.get(0, 1) * (refVelocity - predVel);
        } else {
            fb = K.get(0, 0) * (refVelocity - predVel);
        }

        lastFeedforward = ff;
        lastFeedback = fb;
        double volts = ff + fb;

        double lo = minOutput;
        double hi = maxOutput;
        double vBat = batteryVolts > 1.0 ? batteryVolts : model.nominalVoltage;
        if (voltageCompensation) {
            lo = Math.max(lo, -vBat);
            hi = Math.min(hi, vBat);
        }

        double clamped = Math.max(lo, Math.min(hi, volts));
        lastSaturated = Math.abs(clamped - volts) > 1e-9;
        lastOutput = clamped;

        if (!voltageCompensation) return clamped;
        return Math.max(-1.0, Math.min(1.0, clamped / vBat));
    }

    public double calculateVelocity(double velocity, double batteryVolts) {
        return calculate(0, velocity, batteryVolts);
    }

    public boolean atGoal(double position, double velocity) {
        if (model.type == MechanismModel.Type.POSITION) {
            boolean profileDone = profile == null || profile.isFinished(profileTime);
            return profileDone
                    && Math.abs(goalPosition - position) < positionTolerance
                    && Math.abs(velocity) < velocityTolerance;
        }
        return Math.abs(goalVelocity - velocity) < velocityTolerance;
    }

    public boolean atVelocity(double velocity, double tolerance) {
        return Math.abs(goalVelocity - velocity) < tolerance;
    }

    public void reset(double position, double velocity) {
        this.refPosition = position;
        this.refVelocity = velocity;
        this.refAccel = 0;
        this.profile = null;
        this.profileTime = 0;
        this.lastOutput = 0;
    }

    public Matrix gains() { return K; }

    public double positionGain() { return K.get(0, 0); }

    public double velocityGain() {
        return model.type == MechanismModel.Type.POSITION ? K.get(0, 1) : K.get(0, 0);
    }

    public double referencePosition() { return refPosition; }

    public double referenceVelocity() { return refVelocity; }

    public double referenceAccel() { return refAccel; }

    public double lastFeedforward() { return lastFeedforward; }

    public double lastFeedback() { return lastFeedback; }

    public double lastOutputVolts() { return lastOutput; }

    public boolean isSaturated() { return lastSaturated; }

    public boolean profileFinished() {
        return profile == null || profile.isFinished(profileTime);
    }

    public double profileDuration() { return profile == null ? 0 : profile.duration(); }

    public MechanismModel model() { return model; }

    public double spectralRadius() {
        return LQR.closedLoopSpectralRadius(A, B, K);
    }

    @Override
    public String toString() {
        return String.format("LQRController[%s, K=[%.3f%s], rho=%.4f]",
                model.type, K.get(0, 0),
                model.type == MechanismModel.Type.POSITION
                        ? String.format(", %.3f", K.get(0, 1)) : "",
                spectralRadius());
    }
}
