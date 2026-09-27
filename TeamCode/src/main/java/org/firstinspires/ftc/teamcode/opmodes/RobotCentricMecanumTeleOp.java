package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

/** Robot-centric mecanum drive, adapted from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html. */
@TeleOp(name = "Robot Centric Mecanum", group = "TeleOp")
public class RobotCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.state = Drivetrain.State.DRIVE;

        telemetry.addLine("Left stick: drive and strafe. Right stick: turn.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
            robot.update();
        }

        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
    }
}
