package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Pathing Tuner", group = "test")
public class PathingTuner extends LinearOpMode {
    public static int SHAPE = 0;
    public static double LENGTH = 48.0;

    public static double V_FORWARD = 62.0;
    public static double V_STRAFE  = 48.0;
    public static double A_FORWARD = 70.0;
    public static double A_STRAFE  = 52.0;
    public static double STALL_RATIO = 3.5;
    public static double SAFETY = 0.85;

    public static double TRACK_WIDTH = 14.0;
    public static double WHEEL_BASE = 13.0;
    public static double MAX_WHEEL_SPEED = 66.0;

    public static boolean OPEN_LOOP = true;

    public static double HEADING_GAIN = 4.0;
    public static double CONTOUR_GAIN = 3.2;
    public static double LAG_GAIN = 0.9;
    public static double ACCEL_LEAD = 0.03;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;

        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        FrictionEllipse ellipse = new FrictionEllipse(
                V_FORWARD, V_STRAFE, A_FORWARD, A_STRAFE,
                MAX_WHEEL_SPEED, (TRACK_WIDTH + WHEEL_BASE) / 2.0, STALL_RATIO);

        Trajectory traj = buildShape(ellipse);
        MecanumKinematics kinematics =
                new MecanumKinematics(TRACK_WIDTH, WHEEL_BASE, MAX_WHEEL_SPEED);

        PathFollower follower = new PathFollower(traj, kinematics)
                .headingGain(OPEN_LOOP ? 0 : HEADING_GAIN)
                .contourGain(OPEN_LOOP ? 0 : CONTOUR_GAIN)
                .lagGain(OPEN_LOOP ? 0 : LAG_GAIN)
                .accelLead(OPEN_LOOP ? 0 : ACCEL_LEAD);

        Pose2d start = traj.poseAt(0);
        robot.drivetrain.setPoseEstimate(start);

        telemetry.addLine("Pathing Tuner");
        telemetry.addData("shape", shapeName());
        telemetry.addData("path length (in)", "%.1f", traj.length());
        telemetry.addData("planned duration (s)", "%.2f", traj.duration());
        telemetry.addData("mode", OPEN_LOOP ? "OPEN LOOP (feedforward only)" : "closed loop");
        telemetry.addData("start pose", start.toString());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();
        follower.reset();

        FollowMetrics diag = new FollowMetrics();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive() && !follower.isFinished()) {
            robot.update();

            PathFollower.Command cmd = follower.update(ROBOT_POSITION, robot.sensors.loopTime);
            diag.observe(cmd, traj, robot.sensors.loopTime);

            robot.drivetrain.setMotorPowers(cmd.powers[0], cmd.powers[2], cmd.powers[3], cmd.powers[1]);

            robot.hardwareQueue.update();

            TelemetryUtil.packet.put("Tuner : s", cmd.s);
            TelemetryUtil.packet.put("Tuner : contour error", cmd.contourError);
            TelemetryUtil.packet.put("Tuner : lag error", cmd.lagError);
            TelemetryUtil.packet.put("Tuner : saturation", cmd.saturation);
            TelemetryUtil.packet.put("Tuner : vx", cmd.vx);
            TelemetryUtil.packet.put("Tuner : vy", cmd.vy);
            TelemetryUtil.packet.put("Tuner : omega", cmd.omega);

            // Planned vs. actual along-track profile. Graph each pair together in Dashboard.
            // Actual values are the localizer's global velocity/accel projected onto the path tangent.
            Vector2 tangent = traj.path().tangent(cmd.s);
            TelemetryUtil.packet.put("Profile : planned v", traj.profile().velocity(cmd.s));
            TelemetryUtil.packet.put("Profile : actual v",
                    ROBOT_GLOBAL_VELOCITY.x * tangent.x + ROBOT_GLOBAL_VELOCITY.y * tangent.y);
            TelemetryUtil.packet.put("Profile : planned a", traj.profile().plannedAccel(cmd.s));
            TelemetryUtil.packet.put("Profile : actual a",
                    ROBOT_GLOBAL_ACCELERATION.x * tangent.x + ROBOT_GLOBAL_ACCELERATION.y * tangent.y);
            TelemetryUtil.packet.put("Profile : planned omega", traj.angularVelocityAt(cmd.s));
            TelemetryUtil.packet.put("Profile : actual omega", ROBOT_GLOBAL_VELOCITY.heading);

            if (follower.hasFaulted()) {
                telemetry.addData("FAULT", follower.fault());
                telemetry.update();
                break;
            }
        }

        robot.drivetrain.stopAllMotors();
        robot.update();

        double elapsed = (System.currentTimeMillis() - startMs) / 1000.0;
        Pose2d end = ROBOT_POSITION;

        while (opModeIsActive()) {
            telemetry.addLine(OPEN_LOOP
                    ? "OPEN LOOP result -- judge the shape, not the error"
                    : "closed loop result");
            telemetry.addData("peak contour error (in)", "%.2f", diag.peakContour());
            telemetry.addData("RMS contour error (in)", "%.2f", diag.rmsContour());
            telemetry.addData("mean contour (bias, in)", "%+.2f", diag.meanContour());
            telemetry.addData("weave (crossings/s)", "%.1f", diag.weaveHz());
            telemetry.addData("lag: accel / cruise (in)", "%.2f / %.2f",
                    diag.meanLagAccel(), diag.meanLagCruise());
            telemetry.addData("saturated", "%.0f%% of loops (peak %.2f)",
                    100.0 * diag.saturatedFraction(), diag.peakSaturation());
            telemetry.addData("planned / actual time (s)", "%.2f / %.2f", traj.duration(), elapsed);
            telemetry.addData("end pose", end.toString());
            telemetry.addData("target end pose", traj.poseAt(traj.length()).toString());
            if (follower.hasFaulted()) telemetry.addData("FAULT", follower.fault());
            telemetry.addLine();
            telemetry.addLine("=== next change ===");
            for (String s : recommend(diag)) telemetry.addLine(s);
            telemetry.update();
            robot.update();
        }
    }

    private static List<String> recommend(FollowMetrics m) {
        List<String> out = new ArrayList<>();

        if (m.saturatedFraction() > 0.10) {
            out.add(String.format(Locale.US,
                    "SATURATED %.0f%% of the run. The plan wants more than the wheels have.",
                    100 * m.saturatedFraction()));
            out.add(String.format(Locale.US,
                    "  -> lower SAFETY to %.2f, or re-measure the ellipse.", Math.max(0.5, SAFETY - 0.1)));
            out.add("  Gains are meaningless until this is clear.");
            return out;
        }

        if (OPEN_LOOP) {
            out.add("Open loop. Judge the SHAPE, not the error.");
            out.add(String.format(Locale.US, "  drift %+.1f in over the run.", m.meanContour()));
            out.add(Math.abs(m.meanContour()) > 4.0
                    ? "  -> large one-sided drift: check TRACK_WIDTH/WHEEL_BASE and odometry first."
                    : "  -> shape looks sane. Set OPEN_LOOP = false and tune HEADING_GAIN next.");
            return out;
        }

        if (m.oscillating()) {
            out.add(String.format(Locale.US,
                    "WEAVING: %.1f sign changes/s at %.2f in RMS. Gain is past the limit.",
                    m.weaveHz(), m.rmsContour()));
            out.add(String.format(Locale.US,
                    "  -> lower CONTOUR_GAIN to %.2f.", CONTOUR_GAIN * 0.7));
            return out;
        }

        if (Math.abs(m.meanContour()) > 0.5 && Math.abs(m.meanContour()) > 0.6 * m.rmsContour()) {
            out.add(String.format(Locale.US,
                    "One-sided offset of %+.2f in -- the error barely changes sign.", m.meanContour()));
            out.add("  This is feedforward or geometry, NOT a gain. Raising CONTOUR_GAIN");
            out.add("  will fight it every loop instead of fixing it.");
            out.add("  -> check strafeGain and the ellipse; then let ILC learn the rest.");
            return out;
        }

        if (m.meanLagAccel() > 2.0 && m.meanLagAccel() > 2.0 * m.meanLagCruise()) {
            out.add(String.format(Locale.US,
                    "Lag appears under acceleration only (%.2f in vs %.2f cruising).",
                    m.meanLagAccel(), m.meanLagCruise()));
            out.add(String.format(Locale.US,
                    "  -> raise ACCEL_LEAD to %.3f. Leave LAG_GAIN alone.",
                    Math.min(0.08, ACCEL_LEAD + 0.01)));
            return out;
        }

        if (m.rmsContour() > 1.0) {
            out.add(String.format(Locale.US,
                    "Tracking is loose (%.2f in RMS) and not weaving -- room to stiffen.", m.rmsContour()));
            out.add(String.format(Locale.US,
                    "  -> raise CONTOUR_GAIN to %.2f and re-run.", CONTOUR_GAIN * 1.3));
            return out;
        }

        out.add(String.format(Locale.US,
                "Converged: %.2f in RMS, %.1f crossings/s, no saturation.", m.rmsContour(), m.weaveHz()));
        out.add("  -> re-run on a harder SHAPE, then enable ILC.");
        return out;
    }

    private Trajectory buildShape(FrictionEllipse ellipse) {
        return PathShapes.build(SHAPE, LENGTH, ellipse, SAFETY);
    }

    private String shapeName() { return PathShapes.name(SHAPE); }
}
