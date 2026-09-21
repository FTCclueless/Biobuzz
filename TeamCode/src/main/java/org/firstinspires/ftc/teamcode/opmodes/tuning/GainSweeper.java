package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Gain Sweeper", group = "test")
public class GainSweeper extends LinearOpMode {
    public enum Target { CONTOUR, HEADING, ACCEL_LEAD }

    public static Target TARGET = Target.HEADING;

    public static double START = 1.0;
    public static double STEP_MULTIPLIER = 1.30;
    public static int MAX_TRIALS = 8;

    public static double ABORT_ERROR = 8.0;
    public static double REPOSITION_TOLERANCE = 4.0;
    public static double MARGIN_FRACTION = 0.55;

    public static int SHAPE = PathShapes.STRAIGHT;
    public static double LENGTH = 48.0;
    // Limits, geometry, and the gains NOT being swept all come from DriveConstants.

    private Robot robot;
    private FrictionEllipse ellipse;
    private MecanumKinematics kinematics;

    private static final class Trial {
        double value;
        FollowMetrics metrics;
        boolean aborted;
    }

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        MergeLocalizer.useCamera = false;

        ellipse = DriveConstants.ellipse();
        kinematics = DriveConstants.kinematics();

        Trajectory forward = PathShapes.build(SHAPE, LENGTH, ellipse, DriveConstants.SAFETY);
        Trajectory back = PathShapes.reverse(forward, ellipse, DriveConstants.SAFETY);
        Pose2d startPose = forward.poseAt(0);

        telemetry.addLine("Gain Sweeper");
        telemetry.addData("sweeping", TARGET);
        telemetry.addData("from", "%.2f, x%.2f per trial, up to %d trials",
                START, STEP_MULTIPLIER, MAX_TRIALS);
        telemetry.addData("path", "%s, %.0f in", PathShapes.name(SHAPE), forward.length());
        telemetry.addLine();
        telemetry.addLine("Place the robot at the path start. It returns itself between trials.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.drivetrain.setPoseEstimate(startPose);
        robot.update();

        List<Trial> trials = new ArrayList<>();
        double value = START;

        for (int i = 0; i < MAX_TRIALS && opModeIsActive(); i++) {
            if (!ensureAtStart(startPose, i)) break;

            Trial t = new Trial();
            t.value = value;
            t.metrics = new FollowMetrics();
            t.aborted = !runTrial(forward, value, t.metrics, i + 1);
            trials.add(t);

            if (t.aborted) break;
            if (t.metrics.oscillating()) break;

            if (i < MAX_TRIALS - 1 && opModeIsActive()) driveBack(back);
            value *= STEP_MULTIPLIER;
        }

        robot.drivetrain.stopAllMotors();
        robot.update();
        report(trials);
    }

    private boolean runTrial(Trajectory traj, double value, FollowMetrics metrics, int index) {
        traj.resetMarkers();

        PathFollower follower = new PathFollower(traj, kinematics)
                .headingGain(TARGET == Target.HEADING ? value : DriveConstants.HEADING_GAIN)
                .contourGain(TARGET == Target.CONTOUR ? value : DriveConstants.CONTOUR_GAIN)
                .lagGain(DriveConstants.LAG_GAIN)
                .accelLead(TARGET == Target.ACCEL_LEAD ? value : DriveConstants.ACCEL_LEAD);

        robot.update();
        follower.reset();

        while (opModeIsActive() && !follower.isFinished()) {
            robot.update();
            PathFollower.Command cmd = follower.update(ROBOT_POSITION, robot.sensors.loopTime);
            metrics.observe(cmd, traj, robot.sensors.loopTime);

            robot.drivetrain.setMotorPowers(cmd.powers[0], cmd.powers[2], cmd.powers[3], cmd.powers[1]);
            robot.hardwareQueue.update();

            telemetry.addData("trial", "%d  %s = %.3f", index, TARGET, value);
            telemetry.addData("live", metrics.summary());
            telemetry.update();

            if (Math.abs(cmd.contourError) > ABORT_ERROR || follower.hasFaulted()) {
                robot.drivetrain.stopAllMotors();
                robot.hardwareQueue.update();
                robot.update();
                return false;
            }
        }

        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
        robot.update();
        settle(0.6);
        return true;
    }

    private void driveBack(Trajectory back) {
        back.resetMarkers();
        PathFollower f = new PathFollower(back, kinematics)
                .headingGain(2.0).contourGain(2.0).lagGain(0.5).accelLead(0.03);
        robot.update();
        f.reset();

        while (opModeIsActive() && !f.isFinished()) {
            robot.update();
            PathFollower.Command cmd = f.update(ROBOT_POSITION, robot.sensors.loopTime);
            robot.drivetrain.setMotorPowers(cmd.powers[0], cmd.powers[2], cmd.powers[3], cmd.powers[1]);
            robot.hardwareQueue.update();
            telemetry.addLine("returning to start...");
            telemetry.update();
            if (f.hasFaulted()) break;
        }
        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
        robot.update();
        settle(0.6);
    }

    private boolean ensureAtStart(Pose2d startPose, int index) {
        if (index == 0) return true;

        double off = Vector2.distance(ROBOT_POSITION.toVec2(), startPose.toVec2());
        if (off <= REPOSITION_TOLERANCE) return true;

        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addLine("PAUSED -- robot did not make it back to the start.");
            telemetry.addData("off by (in)", "%.1f", off);
            telemetry.addLine();
            telemetry.addLine("Reposition it on the start mark, then press A on gamepad 1.");
            telemetry.addLine("Stop the op-mode if you would rather keep the results so far.");
            telemetry.update();
            robot.update();
        }
        if (!opModeIsActive()) return false;

        robot.drivetrain.setPoseEstimate(startPose);
        settle(0.4);
        return true;
    }

    private void settle(double seconds) {
        long start = System.nanoTime();
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            robot.drivetrain.stopAllMotors();
            robot.hardwareQueue.update();
            robot.update();
        }
    }

    private void report(List<Trial> trials) {
        Trial critical = null;
        Trial bestStable = null;
        for (Trial t : trials) {
            boolean bad = t.aborted || t.metrics.oscillating();
            if (bad && critical == null) critical = t;
            if (!bad && (bestStable == null || t.metrics.cost() < bestStable.metrics.cost())) {
                bestStable = t;
            }
        }

        while (opModeIsActive()) {
            telemetry.addData("swept", TARGET);
            telemetry.addLine();
            for (Trial t : trials) {
                telemetry.addLine(String.format(Locale.US, "%.3f  %s%s",
                        t.value, t.metrics.summary(),
                        t.aborted ? "  ABORTED" : (t.metrics.oscillating() ? "  <-- WEAVING" : "")));
            }
            telemetry.addLine();

            if (critical != null) {
                double recommended = critical.value * MARGIN_FRACTION;
                telemetry.addLine(String.format(Locale.US,
                        "critical value  %.3f  (%s)", critical.value,
                        critical.aborted ? "aborted on error" : "started weaving"));
                telemetry.addLine(String.format(Locale.US,
                        "RECOMMENDED     %.3f   (%.0f%% of critical)",
                        recommended, 100 * MARGIN_FRACTION));
                telemetry.addLine("  The gap is the stability margin. Do not close it.");
            } else if (bestStable != null) {
                telemetry.addLine(String.format(Locale.US,
                        "No oscillation up to %.3f -- the sweep never found the limit.",
                        trials.get(trials.size() - 1).value));
                telemetry.addLine(String.format(Locale.US,
                        "  best so far %.3f at %s", bestStable.value, bestStable.metrics.summary()));
                telemetry.addLine("  -> raise START or MAX_TRIALS and sweep again.");
            } else {
                telemetry.addLine("No usable trials. Check the path fits and the robot is placed right.");
            }
            telemetry.update();
            robot.update();
        }
    }
}
