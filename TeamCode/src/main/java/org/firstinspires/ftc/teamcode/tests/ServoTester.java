package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    public static boolean usePosition = false, useAngleDeg = false;
    public static double position = 0.5, angle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);

        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<nPriorityServo> servos = new ArrayList<>();

        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        int servoSize = 0;

        // getting number of servos we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof nPriorityServo) {
                servos.add((nPriorityServo) device);
                servoSize++;
            }
        }

        double[] servoPos = new double[servoSize];
        for (int i = 0; i < servoSize; i ++){
            servoPos[i] = servos.get(i).basePos;
        }

        int servoIndex = 0;
        double numLoops = 0;
        double totalTime = 0;

        waitForStart();
        while (!isStopRequested()) {
            START_LOOP();
            robot.sensors.update();
            hardwareQueue.update();

            numLoops ++;

            double speed = gamepad1.right_trigger >= 0.7 ? 0.01 : gamepad1.right_trigger >= 0.4 ? 0.005 : gamepad1.right_trigger >= 0.1 ? 0.003 : 0.001;
            if (gamepad1.x) {
                servoPos[servoIndex] += speed;
            }
            if (gamepad1.b){
                servoPos[servoIndex] -= speed;
            }
            if (gamepad1.right_bumper){
                servoPos[servoIndex] = servos.get(servoIndex).basePos;
            }

            servoPos[servoIndex] = Utils.minMaxClip(servoPos[servoIndex], 0.0, 1.0);

            // figuring out time to set servo pos
            long start = System.nanoTime();

            if (useAngleDeg) {
                servos.get(servoIndex).setTargetAngle(Math.toRadians(angle), 1.0);
            } else {
                servos.get(servoIndex).setTargetPos(usePosition ? position : servoPos[servoIndex], 1.0);
            }

            double elapsedTime = (System.nanoTime()-start)/1000000000.0;
            totalTime += elapsedTime;

            // incrementing / decrementing servoIndex
            if (buttonY.isClicked(gamepad1.y)) {
                servoIndex += 1;
            }

            if (buttonA.isClicked(gamepad1.a)) {
                servoIndex -= 1;
            }

            // if the servoIndex exceeds servoSize wrap around
            servoIndex = (servoIndex + servoSize) % servoSize;

            telemetry.addData("if the servo is reversed, the pos will be reversed too", "lol");
            telemetry.addData("servoName", servos.get(servoIndex).name);
            telemetry.addData("servoIndex", servoIndex);
            telemetry.addData("servoPos", servoPos[servoIndex]);
            telemetry.addData("servoAngle", servos.get(servoIndex).getCurrentAngle());
            telemetry.addData("targetAngle", servos.get(servoIndex).getTargetAngle());
            telemetry.addData("averageServoTime", totalTime/numLoops);
            telemetry.addData("adjustment speed", speed);

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }
    }
}