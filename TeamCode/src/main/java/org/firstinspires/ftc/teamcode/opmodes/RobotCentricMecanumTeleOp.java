package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

/** Robot-centric mecanum drive, adapted from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html. */
@TeleOp(name = "Robot Centric Mecanum", group = "TeleOp")
public class RobotCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.state = Drivetrain.State.DRIVE;

        telemetry.addLine("Left stick: drive and strafe. Right stick: turn.");
        telemetry.addLine("Options/Start: reset Pinpoint pose to (0, 0, 0).");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        long lastLoopStartNanos = System.nanoTime();
        boolean wasResetPressed = false;
        while (opModeIsActive()) {
            long loopStartNanos = System.nanoTime();
            double loopTimeSec = (loopStartNanos - lastLoopStartNanos) / 1e9;
            lastLoopStartNanos = loopStartNanos;

            boolean resetPressed = gamepad1.options;
            if (resetPressed && !wasResetPressed) {
                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
            }
            wasResetPressed = resetPressed;

            pinpoint.update();
            double headingRad = pinpoint.getHeading();
            double headingDeg = Math.toDegrees(headingRad);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double leftFront = (y + x + rx) / denominator;
            double leftRear = (y - x + rx) / denominator;
            double rightFront = (y - x - rx) / denominator;
            double rightRear = (y + x - rx) / denominator;

            // Drivetrain already configures motor directions and the hardware queue.
            robot.drivetrain.setMotorPowers(leftFront, leftRear, rightRear, rightFront);

            telemetry.addData("Loop time (s)", "%.4f", loopTimeSec);
            telemetry.addData("Loop time (ms)", "%.1f", loopTimeSec * 1000);
            telemetry.addData("Pinpoint heading (rad)", "%.3f", headingRad);
            telemetry.addData("Pinpoint heading (deg)", "%.1f", headingDeg);
            telemetry.addLine("Options/Start: reset pose");

            TelemetryUtil.packet.put("TeleOp : Loop time (s)", loopTimeSec);
            TelemetryUtil.packet.put("TeleOp : Loop time (ms)", loopTimeSec * 1000);
            TelemetryUtil.packet.put("TeleOp : Pinpoint heading (rad)", headingRad);
            TelemetryUtil.packet.put("TeleOp : Pinpoint heading (deg)", headingDeg);
            robot.update();
            telemetry.update();
        }

        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
    }
}
