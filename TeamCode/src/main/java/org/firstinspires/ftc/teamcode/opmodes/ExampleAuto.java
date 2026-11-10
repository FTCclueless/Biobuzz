package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name = "Example Auto", group = "Auto")
public class ExampleAuto extends LinearOpMode {
    public static double SIZE = 36.0;
    public static long PAUSE_MS = 600;

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        Path out = line(new Pose(0, 0), new Pose(SIZE, 0)).constant(0);
        Path around = curve(new Pose(SIZE, 0), new Pose(SIZE + 12, SIZE * 0.5), new Pose(SIZE, SIZE)).tangent();
        Path home = curve(new Pose(SIZE, SIZE), new Pose(SIZE * 0.5, SIZE + 12), new Pose(0, 0)).linear(Math.PI / 2, 0);

        telemetry.addLine("Example Auto");
        telemetry.update();
        while (opModeInInit()) robot.update();
        if (isStopRequested()) return;

        LogUtil.init();
        long startedAt = System.currentTimeMillis();
        follow(out);
        follow(around);
        robot.waitFor(PAUSE_MS);
        follow(home);
        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();

        telemetry.addData("done in (s)", "%.1f", (System.currentTimeMillis() - startedAt) / 1000.0);
        telemetry.addData("ended at", Globals.ROBOT_POSITION.toString());
        telemetry.update();
    }

    private void follow(Path path) {
        robot.drivetrain.follow(path);
        robot.waitWhile(robot.drivetrain::isBusy);
    }
}
