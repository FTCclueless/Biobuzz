package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(name = "Intake Sensor Test", group = "test")
public class IntakeSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        Intake intake = robot.intake;

        for (Intake.Side s : Intake.Side.values()) {
            telemetry.addData(s + " sensor", "%s%s", intake.getSensorKind(s),
                    intake.hasColorSensor(s) ? " (color)" : "");
        }
        telemetry.addLine("A intake   B stop   X eject   Y feed   dpad up: pollen retrieval");
        telemetry.addLine("dpad left/right: force side   bumpers: ball count -1 / +1");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean lastLB = false, lastRB = false, lastUp = false;
        while (opModeIsActive()) {
            if (gamepad1.a) intake.intake();
            if (gamepad1.b) intake.stop();
            if (gamepad1.x) intake.eject();
            if (gamepad1.y) intake.feed();
            if (gamepad1.dpad_up && !lastUp) intake.startPollenRetrieval();
            if (gamepad1.dpad_left) intake.setSide(Intake.Side.LEFT);
            if (gamepad1.dpad_right) intake.setSide(Intake.Side.RIGHT);
            if (gamepad1.left_bumper && !lastLB) intake.setBallCount(intake.getBallCount() - 1);
            if (gamepad1.right_bumper && !lastRB) intake.setBallCount(intake.getBallCount() + 1);
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;
            lastUp = gamepad1.dpad_up;

            robot.sensors.update();
            intake.update();
            robot.hardwareQueue.update();
            TelemetryUtil.sendTelemetry();

            telemetry.addData("state / side", "%s / %s", intake.state, intake.getSide());
            telemetry.addData("BALLS HELD", "%d / %d%s", intake.getBallCount(), Intake.CAPACITY,
                    intake.isFull() ? "  FULL (intakes lifted)" : "");
            for (Intake.Side s : Intake.Side.values()) {
                telemetry.addData(s.toString(), "%s  dist %.2f cm  color %s",
                        intake.isBallPresent(s) ? "BALL" : "----",
                        intake.getSensorDistanceCm(s), intake.getBallColor(s));
            }
            telemetry.addData("PRESENT_CM", "%.2f", Intake.PRESENT_CM);
            telemetry.addData("flower angle (rad)", "%.2f", intake.getFlowerAngle());
            telemetry.addData("alliance", Globals.isRed ? "RED (rejects BLUE nectar)"
                    : "BLUE (rejects RED nectar)");
            telemetry.update();
        }
    }
}
