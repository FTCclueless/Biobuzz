package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector3;

import java.util.Locale;

@Config
@TeleOp(name = "Shooter Tester", group = "test")
public class ShooterTester extends LinearOpMode {
    // Field frame: (0, 0) is the field center, inches; heading in degrees.
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING_DEG = 0;

    public static boolean RED_HIVE = true;
    public static int CELL_SIDE = 0; // +1 = +x cell, -1 = -x cell, 0 = whichever is nearer

    public static double FLYWHEEL_VELOCITY = 300;
    public static double HOOD_LAUNCH_DEG = 45;
    public static boolean TRACK_HIVE = true;
    public static double TURRET_DEG = 0;
    public static boolean BLOCKER_OPEN = false;
    public static boolean FEED = false;
    public static boolean DRIVE = true;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        Shooter shooter = robot.shooter;
        shooter.setManual(true);

        while (opModeInInit()) {
            robot.drivetrain.setPoseEstimate(startPose());
            robot.update();
            telemetry.addLine("Shooter Tester. Keep the robot still: odometry is zeroed at init.");
            telemetry.addLine("Place it at START_X / START_Y / START_HEADING_DEG (field center = 0, 0).");
            telemetry.addLine("A spin up   B stop   hold RT: open blocker + feed   X: reset pose to start");
            telemetry.addLine("dpad up: log MADE   dpad down: log MISSED (Logcat tag ShotLog)");
            telemetry.addData("start pose", startPose());
            telemetry.update();
        }

        boolean spinning = false, lastUp = false, lastDown = false;
        String lastLog = "";
        while (opModeIsActive()) {
            if (gamepad1.a) spinning = true;
            if (gamepad1.b) spinning = false;
            if (gamepad1.x) robot.drivetrain.setPoseEstimate(startPose());
            boolean firing = gamepad1.right_trigger > 0.5;

            Vector3 cell = CELL_SIDE == 0 ? BiobuzzField.nearestHiveCell(RED_HIVE, ROBOT_POSITION)
                    : BiobuzzField.hiveCell(RED_HIVE, CELL_SIDE);
            shooter.targetOverride = cell;
            shooter.turretTrackInManual = TRACK_HIVE;
            if (!TRACK_HIVE) shooter.turret.setTargetAngle(Math.toRadians(TURRET_DEG));

            double hoodServo = Utils.minMaxClip(
                    (Math.toRadians(HOOD_LAUNCH_DEG) - Shooter.hoodSweep) * Shooter.hoodGearRatio, 0, 1.6);
            shooter.flywheel.setTargetVelocity(spinning ? FLYWHEEL_VELOCITY : 0);
            shooter.setHoodAngle(hoodServo);
            shooter.setShooterBlocker(!(BLOCKER_OPEN || firing));
            if (FEED || firing) robot.intake.feed(); else robot.intake.stop();
            if (DRIVE) robot.drivetrain.drive(gamepad1);

            robot.update();

            double dist = Math.hypot(cell.x - ROBOT_POSITION.x, cell.y - ROBOT_POSITION.y);
            double vel = shooter.flywheel.getFilteredVelocity();
            double battery = robot.sensors.getVoltage();
            if (gamepad1.dpad_up && !lastUp) lastLog = logShot(true, dist, vel, battery);
            if (gamepad1.dpad_down && !lastDown) lastLog = logShot(false, dist, vel, battery);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            TelemetryUtil.packet.put("ShooterTester : target vel", spinning ? FLYWHEEL_VELOCITY : 0);
            TelemetryUtil.packet.put("ShooterTester : actual vel", vel);
            telemetry.addData("target cell", "%s hive, %s side  (%.1f, %.1f)", RED_HIVE ? "RED" : "BLUE",
                    cell.x >= 0 ? "+x" : "-x", cell.x, cell.y);
            telemetry.addData("DISTANCE to cell (in)", "%.1f", dist);
            telemetry.addData("pose", ROBOT_POSITION);
            telemetry.addData("flywheel (in/s)", "%.0f / %.0f  %s", vel, spinning ? FLYWHEEL_VELOCITY : 0,
                    spinning && shooter.atVel() ? "AT SPEED" : "");
            telemetry.addData("hood launch (deg)", "%.1f  (servo %.3f)", HOOD_LAUNCH_DEG, hoodServo);
            telemetry.addData("turret (deg)", "%.1f / %.1f%s", Math.toDegrees(shooter.turret.getAngle()),
                    Math.toDegrees(shooter.turret.getTargetAngle()), shooter.turret.atLimit() ? "  AT LIMIT" : "");
            telemetry.addData("shots detected", shooter.flywheel.getTotalShots());
            telemetry.addData("battery (V)", "%.2f", battery);
            telemetry.addData("last logged", lastLog);
            telemetry.update();
        }
    }

    private static Pose2d startPose() {
        return new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private static String logShot(boolean made, double dist, double vel, double battery) {
        String line = String.format(Locale.US, "%s,dist=%.1f,targetVel=%.0f,actualVel=%.0f,hoodDeg=%.1f,battery=%.2f",
                made ? "MADE" : "MISSED", dist, FLYWHEEL_VELOCITY, vel, HOOD_LAUNCH_DEG, battery);
        Log.i("ShotLog", line);
        return line;
    }
}
