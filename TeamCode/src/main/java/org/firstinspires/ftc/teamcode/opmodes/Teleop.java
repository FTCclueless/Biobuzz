package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap, true);

        robot.setStopChecker(this::isStopRequested);


        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle lt1 = new ButtonToggle();
        ButtonToggle rt1 = new ButtonToggle();
        ButtonToggle back1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();
        ButtonToggle back2 = new ButtonToggle();
        ButtonToggle h2 = new ButtonToggle();
        ButtonToggle v2 = new ButtonToggle();
        ButtonToggle lb2 = new ButtonToggle();
        ButtonToggle rb2 = new ButtonToggle();
        ButtonToggle guide2 = new ButtonToggle();

        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            robot.sensors.update();
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }

        while (!isStopRequested()) {
            robot.update();

            if (back2.isClicked(gamepad2.back)) {
                isRed = !isRed;
            }


            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.update();
        }

        Globals.AUTO_ENDING_POSE = new Pose(Globals.ROBOT_POSITION.x(), Globals.ROBOT_POSITION.y(), Globals.ROBOT_POSITION.heading());
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = new Pose(Globals.ROBOT_POSITION.x(), Globals.ROBOT_POSITION.y(), Globals.ROBOT_POSITION.heading());
            return true;
        });
    }
}