package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
public class Drivetrain {
    public static double targetHeading = 135;
    public static double headingLockDeadzone = 2.5;
    public static PID headingPID = new PID(0.4, 0.0, 0.002);
    public static double HEADING_K_STATIC = 0.15;
    public static double ACCEL_FILTER = 0.3;

    public final Follower follower;
    public Vision vision;
    private final Robot robot;

    private double[][] pathDrawPoints;
    private Pose2d lastGlobalVelocity = null;
    private boolean wasLocking = false;

    public Drivetrain(Robot robot, Vision vision) {
        this.robot = robot;
        this.vision = vision;
        follower = Constants.create(robot.hardwareMap);
    }

    public void update() {
        if (!DRIVETRAIN_ENABLED) follower.stop();
        follower.update();
        publishState();
        updateTelemetry();
    }

    private void publishState() {
        Pose p = follower.pose();
        Velocity v = follower.velocity();
        double c = Math.cos(p.heading()), s = Math.sin(p.heading());
        Pose2d global = new Pose2d(v.vx, v.vy, v.omega);

        ROBOT_POSITION = new Pose2d(p.x(), p.y(), p.heading());
        Globals.ROBOT_GLOBAL_VELOCITY = global;
        Globals.ROBOT_VELOCITY = new Pose2d(v.vx * c + v.vy * s, -v.vx * s + v.vy * c, v.omega);

        double dt = robot.sensors.loopTime;
        if (lastGlobalVelocity != null && dt > 0) {
            Pose2d a = Globals.ROBOT_GLOBAL_ACCELERATION;
            Globals.ROBOT_GLOBAL_ACCELERATION = new Pose2d(
                    a.x + ((v.vx - lastGlobalVelocity.x) / dt - a.x) * ACCEL_FILTER,
                    a.y + ((v.vy - lastGlobalVelocity.y) / dt - a.y) * ACCEL_FILTER,
                    a.heading + ((v.omega - lastGlobalVelocity.heading) / dt - a.heading) * ACCEL_FILTER);
        }
        lastGlobalVelocity = global;
    }

    public void follow(Path path) {
        pathDrawPoints = DashboardUtil.samplePath(path);
        follower.follow(path);
    }

    public boolean isBusy() { return follower.isBusy(); }

    public void goToPoint(Pose2d target) {
        pathDrawPoints = null;
        follower.hold(toPedro(target));
    }

    public void stop() { follower.stop(); }

    public void setPoseEstimate(Pose2d pose) {
        follower.setPose(toPedro(pose));
        ROBOT_POSITION = pose.clone();
        lastGlobalVelocity = null;
        Globals.ROBOT_GLOBAL_ACCELERATION = new Pose2d(0, 0, 0);
        LogUtil.drivePositionReset = true;
    }

    public Pose2d getPoseEstimate() { return ROBOT_POSITION; }

    public static Pose toPedro(Pose2d p) { return new Pose(p.x, p.y, p.heading); }

    public void drive(Gamepad gamepad) { drive(gamepad, false); }

    public void drive(Gamepad gamepad, boolean lockHeading) {
        double forward = smoothControls(-gamepad.left_stick_y);
        double strafe = smoothControls(-gamepad.left_stick_x);
        if (Math.hypot(forward, strafe) <= 0.05) forward = strafe = 0;

        double turn;
        if (lockHeading) {
            if (!wasLocking) headingPID.resetIntegral();
            wasLocking = true;
            double error = AngleUtil.clipAngle(Math.toRadians(targetHeading) * (Globals.isRed ? 1 : -1)
                    - ROBOT_POSITION.heading);
            if (Math.abs(error) < Math.toRadians(headingLockDeadzone)) {
                turn = 0;
                headingPID.resetIntegral();
            } else {
                turn = headingPID.update(error, -1, 1) + HEADING_K_STATIC * Math.signum(error);
            }
        } else {
            wasLocking = false;
            turn = smoothControls(-gamepad.right_stick_x);
        }
        follower.manual(forward, strafe, turn);
    }

    public double smoothControls(double value) {
        return 0.7 * Math.tan(0.96 * value);
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Drivetrain : mode", follower.mode());
        LogUtil.driveState.set(follower.mode().toString());
        if (follower.following()) {
            TelemetryUtil.packet.put("Path : distance to end", follower.distanceToEndpoint());
            TelemetryUtil.packet.put("Path : completion", follower.completion());
            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            if (pathDrawPoints != null) DashboardUtil.drawSampledPath(canvas, pathDrawPoints);
            Pose c = follower.closestPose();
            DashboardUtil.drawRobot(canvas, new Pose2d(c.x(), c.y(), c.heading()), "#8000ff");
        }
    }
}
