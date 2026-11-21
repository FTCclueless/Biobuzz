package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.ClusterChaser;

/**
 * Vision-only test: chase clusters of POLLEN / own NECTAR for the whole auto. See ClusterChaser.
 * Init: gamepad1 X = blue, B = red (decides which NECTAR counts as ours).
 */
@Config
@Autonomous(name = "Cluster Chase Auto", group = "Auto")
public class ClusterChaseAuto extends LinearOpMode {
    public static double AUTO_SECONDS = 28;
    public static int MAX_CLUSTERS = 3;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap, true);
        if (robot.balls == null) throw new IllegalStateException("Cluster Chase needs Globals.USE_VISION = true (Limelight wired)");
        robot.setStopChecker(this::isStopRequested);

        boolean red = Globals.isRed;
        Boolean poseSetFor = null;
        while (opModeInInit()) {
            if (gamepad1.x) red = false;
            if (gamepad1.b) red = true;
            if (poseSetFor == null || poseSetFor != red) {
                Globals.isRed = red;
                BiobuzzField.setSide(red ? BiobuzzField.Side.RED : BiobuzzField.Side.BLUE);
                robot.drivetrain.setPoseEstimate(BiobuzzField.field().start());
                poseSetFor = red;
            }

            robot.update();
            telemetry.addData("Alliance (X blue / B red)", red ? "RED" : "BLUE");
            telemetry.addData("Limelight connected", robot.balls.getLimelight().isConnected());
            telemetry.addData("Balls tracked", robot.balls.getTrackedBalls().size());
            telemetry.update();
        }
        if (isStopRequested()) { robot.balls.stop(); return; }

        LogUtil.init();
        new ClusterChaser(robot).run(MAX_CLUSTERS, (long) (AUTO_SECONDS * 1000));

        robot.balls.stop();
        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
    }
}
