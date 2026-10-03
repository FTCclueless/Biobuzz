package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.Locale;

/**
 * Checks that the odometry believes the truth. Everything the pathing system does rests on this:
 * a follower cannot drive a path more accurately than the localizer can measure it.
 *
 * Drive the robot around by hand or with the sticks, return it to exactly where it started, and
 * compare. A good setup comes back within about an inch and a couple of degrees after a lap of
 * the field.
 *
 * What the error tells you:
 *   position drifts but heading is right   -> wheel diameter or odo pod placement
 *   heading drifts                         -> track width, or the IMU/Pinpoint needs a reset
 *   error grows only while spinning        -> odo pod lateral offset
 *   error jumps suddenly                   -> a pod is slipping or a cable is catching
 */
@Config
@TeleOp(name = "Localization Test", group = "test")
public class LocalizationTest extends LinearOpMode {
    /** Drive speed multiplier for the sticks. */
    public static double DRIVE_POWER = 0.5;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        Pose2d origin = new Pose2d(0, 0, 0);
        robot.drivetrain.setPoseEstimate(origin);

        telemetry.addLine("Localization Test");
        telemetry.addLine();
        telemetry.addLine("Mark the robot's exact starting spot on the tile first.");
        telemetry.addLine("Drive a lap, return to the mark, and read the error.");
        telemetry.addLine("Left stick drives, right stick turns. A resets the pose to 0,0,0.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double maxSpeed = 0;
        double distanceTravelled = 0;
        Pose2d last = Globals.ROBOT_POSITION.clone();

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.a) {
                robot.drivetrain.setPoseEstimate(origin);
                distanceTravelled = 0;
            }

            robot.drivetrain.setMoveVector(
                    new Vector2(-gamepad1.left_stick_y * DRIVE_POWER,
                            -gamepad1.left_stick_x * DRIVE_POWER),
                    -gamepad1.right_stick_x * DRIVE_POWER);

            Pose2d now = Globals.ROBOT_POSITION;
            distanceTravelled += Math.hypot(now.x - last.x, now.y - last.y);
            last = now.clone();

            double speed = Math.hypot(Globals.ROBOT_VELOCITY.x, Globals.ROBOT_VELOCITY.y);
            maxSpeed = Math.max(maxSpeed, speed);

            telemetry.addData("pose", "x %.2f  y %.2f  h %.1f deg",
                    now.x, now.y, Math.toDegrees(now.heading));
            telemetry.addLine();
            telemetry.addData("error from start", String.format(Locale.US,
                    "%.2f in,  %.1f deg",
                    Math.hypot(now.x, now.y), Math.toDegrees(now.heading)));
            telemetry.addData("distance driven (in)", "%.0f", distanceTravelled);
            telemetry.addData("drift per 100 in", "%.2f in",
                    distanceTravelled > 1 ? Math.hypot(now.x, now.y) / distanceTravelled * 100 : 0);
            telemetry.addLine();
            telemetry.addData("speed now / peak (in/s)", "%.1f / %.1f", speed, maxSpeed);
            telemetry.addLine("peak speed is a sanity check on V_FORWARD in DriveConstants");
            telemetry.update();
        }
    }
}
