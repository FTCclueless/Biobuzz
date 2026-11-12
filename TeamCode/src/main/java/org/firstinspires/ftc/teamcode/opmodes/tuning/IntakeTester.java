package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(name = "Intake Tester", group = "test")
public class IntakeTester extends LinearOpMode {
    public static double MOTOR_POWER = 0;
    public static double TRANSFER_POWER = 0;
    public static double FLOWER_POWER = 0;
    public static boolean DRIVE = true;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        //robot.shooter.setManual(true);
        Intake intake = robot.intake;

        telemetry.addLine("Intake Tester: set values in Dashboard > IntakeTester");
        telemetry.addLine("Raw hardware control, no intake logic. Ball counting still runs.");
        telemetry.addLine("Hold A: motor at +1   hold X: motor at -1   Y: pivot to down angle B: active intake pivot side to front");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            intake.state = Intake.State.TEST;
            double power = gamepad1.a ? 1 : gamepad1.x ? -1 : MOTOR_POWER;
            double transferPow = gamepad1.y ? 1 : gamepad1.x ? -1 : TRANSFER_POWER;
            intake.intakeMotor.setTargetPower(power);
            intake.transferMotor.setTargetPower(transferPow);

            if(gamepad1.left_bumper)
            {
                intake.backPivot.setTargetPos(Intake.backPivotDownPos);
            }
            else
            {
                intake.backPivot.setTargetPos(Intake.backPivotUpPos);

            }

            //intake.pivot.setTargetAngle(pivot);
            intake.flowerMechanismServo.setTargetPower(FLOWER_POWER);
            if (DRIVE) robot.drivetrain.drive(gamepad1);

            robot.update();

            telemetry.addData("motor power", "%.2f", power);
            telemetry.addData("transfer power", "%.2f", transferPow);

            /*telemetry.addData("pivot angle", "%.3f  (up %.3f, down %.3f)", pivot,
                    Intake.intakePivotUpPosition, Intake.intakePivotDownPosition);*/
            telemetry.addData("flower power / angle (rad)", "%.2f / %.2f", FLOWER_POWER, intake.getFlowerAngle());
            //telemetry.addData("balls held", intake.getBallCount());
            for (Intake.Side s : Intake.Side.values()) {
                telemetry.addData(s + " sensor", "%s  %.2f cm  %s", intake.isBallPresent(s) ? "BALL" : "----",
                        intake.getSensorDistanceCm(s), intake.getBallColor(s));
            }
            telemetry.update();
        }
    }
}
