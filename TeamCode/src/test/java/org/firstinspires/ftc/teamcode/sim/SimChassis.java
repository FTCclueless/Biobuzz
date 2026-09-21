package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Utils;

public final class SimChassis {
    private double x, y, heading;
    private double vx, vy, omega;

    private final MecanumKinematics kinematics;
    private final FrictionEllipse grip;

    public double motorTau = 0.085;
    public double yawTau = 0.11;
    public double strafeEfficiency = 0.88;
    public double maxAngularAccel = 14.0;
    public double nominalVoltage = 12.5;

    private double voltage = 12.5;
    private double tractionScale = 1.0;
    private double pushX = 0, pushY = 0;

    private double lastSlipMagnitude = 0;
    private boolean slippingNow = false;
    private int slipLoops = 0;
    private int totalLoops = 0;

    public SimChassis(MecanumKinematics kinematics, FrictionEllipse grip, Pose2d start) {
        this.kinematics = kinematics;
        this.grip = grip;
        this.x = start.x;
        this.y = start.y;
        this.heading = start.heading;
    }

    public void setVoltage(double v) { this.voltage = v; }

    public double voltage() { return voltage; }

    public void setTractionScale(double f) { this.tractionScale = f; }

    public void push(double dvxField, double dvyField) {
        pushX += dvxField;
        pushY += dvyField;
    }

    public void step(double[] powers, double dt) {
        totalLoops++;

        double voltageFactor = voltage / nominalVoltage;
        double wheelFree = kinematics.maxWheelSpeed() * voltageFactor;
        double fl = Utils.minMaxClip(powers[0], -1, 1) * wheelFree;
        double fr = Utils.minMaxClip(powers[1], -1, 1) * wheelFree;
        double bl = Utils.minMaxClip(powers[2], -1, 1) * wheelFree;
        double br = Utils.minMaxClip(powers[3], -1, 1) * wheelFree;

        double[] want = kinematics.toTwist(fl, fr, bl, br);
        double wantVx = want[0];
        double wantVy = want[1] * strafeEfficiency;
        double wantOmega = want[2];

        double axReq = (wantVx - vx) / motorTau;
        double ayReq = (wantVy - vy) / motorTau;
        double alphaReq = (wantOmega - omega) / yawTau;

        double aMag = Math.hypot(axReq, ayReq);
        double slip = 0;
        if (aMag > 1e-9) {
            double beta = Math.atan2(ayReq, axReq);
            double available = grip.maxAccel(beta) * tractionScale;
            if (aMag > available) {
                double k = available / aMag;
                slip = aMag - available;
                axReq *= k;
                ayReq *= k;
            }
        }
        lastSlipMagnitude = slip;
        slippingNow = slip > 1.0;
        if (slippingNow) slipLoops++;

        alphaReq = Utils.minMaxClip(alphaReq, -maxAngularAccel, maxAngularAccel);

        vx += axReq * dt;
        vy += ayReq * dt;
        omega += alphaReq * dt;

        if (pushX != 0 || pushY != 0) {
            double c = Math.cos(heading), s = Math.sin(heading);
            vx += pushX * c + pushY * s;
            vy += -pushX * s + pushY * c;
            pushX = 0;
            pushY = 0;
        }

        double dtheta = omega * dt;
        double c = Math.cos(heading), s = Math.sin(heading);
        double dxBody, dyBody;
        if (Math.abs(dtheta) < 1e-9) {
            dxBody = vx * dt;
            dyBody = vy * dt;
        } else {
            double sinc = Math.sin(dtheta) / dtheta;
            double cosc = (1 - Math.cos(dtheta)) / dtheta;
            dxBody = (vx * sinc - vy * cosc) * dt;
            dyBody = (vx * cosc + vy * sinc) * dt;
        }
        x += dxBody * c - dyBody * s;
        y += dxBody * s + dyBody * c;
        heading = Utils.headingClip(heading + dtheta);
    }

    public Pose2d truePose() { return new Pose2d(x, y, heading); }

    public double trueSpeed() { return Math.hypot(vx, vy); }

    public double vx() { return vx; }

    public double vy() { return vy; }

    public double omega() { return omega; }

    public boolean isSlipping() { return slippingNow; }

    public double slipMagnitude() { return lastSlipMagnitude; }

    public double slipFraction() { return totalLoops == 0 ? 0 : (double) slipLoops / totalLoops; }
}
