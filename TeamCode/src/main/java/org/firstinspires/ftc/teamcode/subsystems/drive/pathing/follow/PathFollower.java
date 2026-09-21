package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow;

import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Utils;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.VelocityProfile;

public final class PathFollower {
    private double contourGain = 3.2;
    private double lagGain = 0.9;
    private double forwardGain = 1.0;
    private double strafeGain = 1.25;
    private double headingGain = 4.0;
    private double accelLead = 0.035;
    private double startVelocityFloor = 6.0;
    private double startFloorDistance = 2.0;
    private double projectionWindow = 12.0;
    private double maxReferenceLead = 6.0;
    private double positionTolerance = 1.0;
    private double headingTolerance = Math.toRadians(4);

    private final Trajectory trajectory;
    private final GuidingVectorField gvf;
    private final MecanumKinematics kinematics;
    private IterativeLearner learner;

    private double s = 0;
    private double sReference = 0;
    private String lastFault = null;
    private final Path.Sample sample = new Path.Sample();
    private final GuidingVectorField.Field field = new GuidingVectorField.Field();
    private double lastContourError = 0;
    private double lastLagError = 0;
    private double lastSaturation = 0;
    private boolean finished = false;

    public PathFollower(Trajectory trajectory, MecanumKinematics kinematics) {
        this.trajectory = trajectory;
        this.kinematics = kinematics;
        this.gvf = new GuidingVectorField(trajectory.path())
                .contourGain(0.9)
                .lagGain(0.25);
    }

    public PathFollower contourGain(double v) { this.contourGain = v; return this; }

    public PathFollower lagGain(double v) { this.lagGain = v; return this; }

    public PathFollower forwardGain(double v) { this.forwardGain = v; return this; }

    public PathFollower strafeGain(double v) { this.strafeGain = v; return this; }

    public PathFollower headingGain(double v) { this.headingGain = v; return this; }

    public PathFollower accelLead(double v) { this.accelLead = v; return this; }

    public PathFollower startVelocityFloor(double v) { this.startVelocityFloor = v; return this; }

    public PathFollower positionTolerance(double v) { this.positionTolerance = v; return this; }

    public PathFollower learner(IterativeLearner l) { this.learner = l; return this; }

    public GuidingVectorField gvf() { return gvf; }

    public static final class Command {
        public final double vx;
        public final double vy;
        public final double omega;
        public final double[] powers;
        public final double s;
        public final double contourError;
        public final double lagError;
        public final double saturation;

        Command(double vx, double vy, double omega, double[] powers, double s,
                double contourError, double lagError, double saturation) {
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            this.powers = powers;
            this.s = s;
            this.contourError = contourError;
            this.lagError = lagError;
            this.saturation = saturation;
        }
    }

    public Command update(Pose2d pose, double dt) {
        if (!MathUtil.allFinite(pose.x, pose.y, pose.heading)
                || !MathUtil.isFinite(dt) || dt <= 0) {
            return fault("non-finite pose or dt from odometry: " + pose + " dt=" + dt);
        }
        Path path = trajectory.path();
        VelocityProfile profile = trajectory.profile();

        s = path.project(pose.x, pose.y, s, projectionWindow, sample);
        trajectory.fireMarkers(s);

        path.sample(s, sample);
        double vRef = profile.velocity(s);
        double aRef = profile.plannedAccel(s);

        if (s < startFloorDistance) {
            vRef = Math.max(vRef, startVelocityFloor);
        }

        sReference = Utils.minMaxClip(sReference + vRef * dt, s, s + maxReferenceLead);
        sReference = Math.min(sReference, path.length());
        gvf.evaluateInto(pose.x, pose.y, sample, sReference, vRef, field);

        double contourError = field.contourError;
        double lagError = field.lagError;

        double speed = vRef + aRef * accelLead;

        double fieldVelX = field.dirX * speed;
        double fieldVelY = field.dirY * speed;
        double corrN = -contourGain * contourError;
        double corrT = lagGain * lagError;

        if (learner != null) {
            corrN += learner.correctionAt(s);
        }

        double desiredX = fieldVelX + sample.nx() * corrN + sample.tx * corrT;
        double desiredY = fieldVelY + sample.ny() * corrN + sample.ty * corrT;

        double ch = Math.cos(pose.heading), sh = Math.sin(pose.heading);
        double vx = (desiredX * ch + desiredY * sh) * forwardGain;
        double vy = (-desiredX * sh + desiredY * ch) * strafeGain;

        double refHeading = trajectory.heading().heading(s, sample);
        double omegaFf = trajectory.angularVelocityAt(s, sample);
        double headingError = Utils.headingClip(refHeading - pose.heading);
        double omega = omegaFf + headingGain * headingError;

        double sat = kinematics.saturation(vx, vy, omega);
        lastSaturation = sat;
        if (sat > 1.0) {
            double tanBx = sample.tx * ch + sample.ty * sh;
            double tanBy = -sample.tx * sh + sample.ty * ch;
            double along = vx * tanBx + vy * tanBy;
            double alongX = tanBx * along, alongY = tanBy * along;
            double perpX = vx - alongX, perpY = vy - alongY;

            double keep = 1.0;
            for (int i = 0; i < 12; i++) {
                if (kinematics.saturation(perpX + alongX * keep,
                        perpY + alongY * keep, omega) <= 1.0) break;
                keep *= 0.7;
            }
            vx = perpX + alongX * keep;
            vy = perpY + alongY * keep;
        }

        if (!MathUtil.allFinite(vx, vy, omega)) {
            return fault(String.format(
                    "non-finite command at s=%.2f: vx=%f vy=%f omega=%f", s, vx, vy, omega));
        }

        double[] powers = kinematics.toPowers(vx, vy, omega);

        if (learner != null) {
            double gvfLateral = gvf.lateralVelocityCommand(field, vRef);
            double pLateral = -contourGain * contourError;
            learner.observe(s, gvfLateral + pLateral, dt);
        }

        lastContourError = contourError;
        lastLagError = lagError;

        double distToEnd = Vector2.distance(pose.toVec2(), path.endPoint());
        double headErr = Math.abs(Utils.headingClip(
                trajectory.heading().heading(path.length()) - pose.heading));
        double endWindow = Math.max(2.0 * positionTolerance, 0.02 * path.length());
        boolean traversed = s >= path.length() - endWindow;
        if (traversed && distToEnd < positionTolerance && headErr < headingTolerance) {
            finished = true;
            trajectory.fireMarkers(path.length());
        }

        return new Command(vx, vy, omega, powers, s, contourError, lagError, sat);
    }

    public boolean isFinished() { return finished; }

    public double arcLength() { return s; }

    public double contourError() { return lastContourError; }

    public double lagError() { return lastLagError; }

    public double saturation() { return lastSaturation; }

    public Trajectory trajectory() { return trajectory; }

    private Command fault(String reason) {
        lastFault = reason;
        return new Command(0, 0, 0, new double[]{0, 0, 0, 0}, s, 0, 0, 0);
    }

    public String fault() { return lastFault; }

    public boolean hasFaulted() { return lastFault != null; }

    public double referenceArcLength() { return sReference; }

    public void reset() {
        s = 0;
        sReference = 0;
        lastFault = null;
        finished = false;
        lastContourError = 0;
        lastLagError = 0;
        trajectory.resetMarkers();
        if (learner != null) learner.startRun();
    }
}
