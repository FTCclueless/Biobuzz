package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name = "Blue Auto", group = "Auto")
public class BlueAuto extends LinearOpMode {
    public static double SIZE = 12;
    public static long PAUSE_MS = 600;

    public static long shootDuration = 1000;

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.isRed = false;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);

        Path test = line(new Pose(0, 0), new Pose(SIZE, 0)).constant(0);
        Path toFlower = line(new Pose(0, 0), new Pose(-SIZE/2+24, 0)).constant(0);
        Path toIntake = line(new Pose(-SIZE/2+24, 0), new Pose(-36, 0)).constant(0);
        Path toGarden = line(new Pose(-36, 0), new Pose(48, 0)).constant(0);
        Path park = line(new Pose(48, 0), new Pose(48, -4*24)).constant(0);

        telemetry.addLine("Example Auto");
        telemetry.update();
        while (opModeInInit()) robot.update();
        if (isStopRequested()) return;

        LogUtil.init();
        long startedAt = System.currentTimeMillis();

        long t = System.currentTimeMillis();
        robot.shooter.setManual(false);

        robot.shooter.reqAim(true);

        shoot(test, false); // fire preloads
        robot.intake.startPollenRetrieval();
        follow(toFlower);
        robot.intake.intake();
        follow(toIntake);
        follow(toGarden);
        robot.intake.feed();
        shoot(park, true);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();

        telemetry.addData("done in (s)", "%.1f", (System.currentTimeMillis() - startedAt) / 1000.0);
        telemetry.addData("ended at", Globals.ROBOT_POSITION.toString());
        telemetry.update();
    }

    private void follow(Path path) {
        robot.drivetrain.follow(path);
        robot.waitWhile(robot.drivetrain::isBusy);
    }

    private void shoot(Path shootMovement, boolean goToShootPos){
        if(goToShootPos) {
            robot.drivetrain.follow(shootMovement);
            robot.waitWhile(() -> robot.drivetrain.follower.isBusy() || robot.shooter.state != Shooter.State.READY);
        } else{
            robot.waitWhile(() -> robot.shooter.state != Shooter.State.READY);
        }

        robot.shooter.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.reqStop(true);
        robot.shooter.reqAim(true);

    }
}
