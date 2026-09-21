package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow;

import org.firstinspires.ftc.teamcode.utils.Vector2;

public final class MecanumKinematics {
    private final double geom;
    private final double maxWheelSpeed;

    public MecanumKinematics(double trackWidth, double wheelBase, double maxWheelSpeed) {
        this.geom = (trackWidth + wheelBase) / 2.0;
        this.maxWheelSpeed = maxWheelSpeed;
    }

    public double maxWheelSpeed() { return maxWheelSpeed; }

    public double geometry() { return geom; }

    public double[] toWheelSpeeds(double vx, double vy, double omega) {
        double r = omega * geom;
        return new double[]{
                vx - vy - r,
                vx + vy + r,
                vx + vy - r,
                vx - vy + r
        };
    }

    public double[] toWheelSpeeds(Vector2 bodyVelocity, double omega) {
        return toWheelSpeeds(bodyVelocity.x, bodyVelocity.y, omega);
    }

    public double[] toTwist(double fl, double fr, double bl, double br) {
        double vx = (fl + fr + bl + br) / 4.0;
        double vy = (-fl + fr + bl - br) / 4.0;
        double omega = (-fl + fr - bl + br) / (4.0 * geom);
        return new double[]{vx, vy, omega};
    }

    public double[] toPowers(double vx, double vy, double omega) {
        double[] w = toWheelSpeeds(vx, vy, omega);
        double peak = 0;
        for (double x : w) peak = Math.max(peak, Math.abs(x));
        double scale = 1.0 / maxWheelSpeed;
        if (peak * scale > 1.0) scale = 1.0 / peak;
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) out[i] = w[i] * scale;
        return out;
    }

    public double saturation(double vx, double vy, double omega) {
        double[] w = toWheelSpeeds(vx, vy, omega);
        double peak = 0;
        for (double x : w) peak = Math.max(peak, Math.abs(x));
        return peak / maxWheelSpeed;
    }
}
