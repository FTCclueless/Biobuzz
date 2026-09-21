package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

public final class FrictionEllipse {
    private final double vForward;
    private final double vStrafe;
    private final double aForward;
    private final double aStrafe;
    private final double wheelMaxSpeed;
    private final double wheelGeom;
    private final double stallRatio;

    public FrictionEllipse(double vForward, double vStrafe, double aForward, double aStrafe,
                           double wheelMaxSpeed, double wheelGeom, double stallRatio) {
        this.vForward = vForward;
        this.vStrafe = vStrafe;
        this.aForward = aForward;
        this.aStrafe = aStrafe;
        this.wheelMaxSpeed = wheelMaxSpeed;
        this.wheelGeom = wheelGeom;
        this.stallRatio = stallRatio;
    }

    public static FrictionEllipse typicalFtc() {
        return new FrictionEllipse(
                62.0,
                48.0,
                70.0,
                52.0,
                66.0,
                (14.0 + 13.0) / 2.0,
                3.5);
    }

    public FrictionEllipse scaled(double f) {
        return new FrictionEllipse(vForward * f, vStrafe * f, aForward * f, aStrafe * f,
                wheelMaxSpeed * f, wheelGeom, stallRatio);
    }

    public double maxForwardSpeed() { return vForward; }

    public double maxStrafeSpeed() { return vStrafe; }

    public double maxForwardAccel() { return aForward; }

    public double wheelGeometry() { return wheelGeom; }

    public double wheelMaxSpeed() { return wheelMaxSpeed; }

    private static double ellipse(double a, double b, double beta) {
        double c = Math.cos(beta) / a;
        double s = Math.sin(beta) / b;
        return 1.0 / Math.sqrt(c * c + s * s);
    }

    public double maxSpeed(double beta) {
        return ellipse(vForward, vStrafe, beta);
    }

    public double maxAccel(double beta) {
        return ellipse(aForward, aStrafe, beta);
    }

    public double maxSpeedForCurvature(double k, double beta) {
        double ak = Math.abs(k);
        if (ak < 1e-9) return maxSpeed(beta);
        double aLat = maxAccel(beta + Math.PI / 2);
        return Math.min(maxSpeed(beta), Math.sqrt(aLat / ak));
    }

    public double maxSpeedFromWheels(double beta, double dThetaDs) {
        double cx = Math.cos(beta);
        double cy = Math.sin(beta);
        double r = wheelGeom * dThetaDs;
        double c1 = Math.abs(cx - cy - r);
        double c2 = Math.abs(cx + cy + r);
        double c3 = Math.abs(cx + cy - r);
        double c4 = Math.abs(cx - cy + r);
        double worst = Math.max(Math.max(c1, c2), Math.max(c3, c4));
        if (worst < 1e-9) return Double.MAX_VALUE;
        return wheelMaxSpeed / worst;
    }

    public double motorAccelLimit(double v, double beta) {
        double vMax = maxSpeed(beta);
        double frac = 1.0 - Utils.minMaxClip(v / vMax, 0, 1);
        return stallRatio * maxAccel(beta) * frac;
    }

    public double[] longitudinalAccelRange(double v, double k, double beta) {
        double aLatMax = maxAccel(beta + Math.PI / 2);
        double used = Utils.minMaxClip(v * v * Math.abs(k) / aLatMax, 0, 1);
        double traction = maxAccel(beta) * MathUtil.safeSqrt(1.0 - used * used);

        double accel = Math.min(traction, motorAccelLimit(v, beta));
        double brake = traction;

        if (accel < 1e-3) accel = 1e-3;
        if (brake < 1e-3) brake = 1e-3;
        return new double[]{-brake, accel};
    }

    public double regimeCrossoverSpeed() {
        return maxSpeed(0) * (1.0 - 1.0 / stallRatio);
    }

    @Override
    public String toString() {
        return String.format(
                "FrictionEllipse[v %.0f/%.0f in/s, a %.0f/%.0f in/s^2, stall x%.1f, crossover %.0f in/s]",
                vForward, vStrafe, aForward, aStrafe, stallRatio, regimeCrossoverSpeed());
    }
}
