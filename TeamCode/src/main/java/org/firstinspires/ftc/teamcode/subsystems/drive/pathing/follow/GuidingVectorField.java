package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow;

import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;

public final class GuidingVectorField {
    private final Path path;
    private double contourGain = 0.9;
    private double lagGain = 0.25;
    private double maxTilt = 1.2;

    public GuidingVectorField(Path path) {
        this.path = path;
    }

    public GuidingVectorField contourGain(double v) { this.contourGain = v; return this; }

    public GuidingVectorField lagGain(double v) { this.lagGain = v; return this; }

    public GuidingVectorField maxTilt(double v) { this.maxTilt = v; return this; }

    public double contourGain() { return contourGain; }

    public double lagGain() { return lagGain; }

    public static final class Field {
        public double dirX;
        public double dirY;
        public double contourError;
        public double lagError;
        public double tilt;
        public double s;

        public Field() {}

        public Vector2 direction() { return new Vector2(dirX, dirY); }
    }

    public Field evaluate(Vector2 robot, double sProj, double sRef, double speed) {
        Path.Sample sm = new Path.Sample();
        path.sample(sProj, sm);
        Field out = new Field();
        evaluateInto(robot.x, robot.y, sm, sRef, speed, out);
        return out;
    }

    public void evaluateInto(double robotX, double robotY, Path.Sample sm,
                             double sRef, double speed, Field out) {
        double contour = (robotX - sm.px) * sm.nx() + (robotY - sm.py) * sm.ny();
        double lag = sRef - sm.s;

        double vRef = Math.max(speed, 4.0);
        double lateralCmd = -contourGain * contour;
        double tangentCmd = vRef + lagGain * lag;

        double tilt = Math.atan2(lateralCmd, Math.max(tangentCmd, 1e-3));
        tilt = Utils.minMaxClip(tilt, -maxTilt, maxTilt);

        double c = Math.cos(tilt), sn = Math.sin(tilt);
        out.dirX = sm.tx * c - sm.ty * sn;
        out.dirY = sm.tx * sn + sm.ty * c;
        out.contourError = contour;
        out.lagError = lag;
        out.tilt = tilt;
        out.s = sm.s;
    }

    public double lateralVelocityCommand(Field f, double speed) {
        return Math.max(speed, 4.0) * Math.tan(f.tilt);
    }

    public Path path() { return path; }
}
