package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
@Autonomous(name = "One Tip Red Auto", group = "Auto")
public class OneTipRedAuto extends LinearOpMode
{
    public static double SIZE = 12;
    public static long PAUSE_MS = 600;

    public static long shootDuration = 1000;

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        BiobuzzField.setSide(BiobuzzField.Side.RED);

        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        Pose2d startBase = BiobuzzField.field().start();
        robot.drivetrain.setPoseEstimate(startBase);
        Pose startPose = new Pose(startBase.x, startBase.y, startBase.heading);

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);


        Path test = line(new Pose(0, 0).plus(startPose), new Pose(SIZE, 0).plus(startPose)).constant(0);
        Path toFlower = line(startPose, Utils.fromPose2d(BiobuzzField.field().flowerSide())).constant(0);

        Path toGardenNew = line(startPose, Utils.fromPose2d(BiobuzzField.field().garden())).constant(0);
        Path toShootPosAfterTip = line(Utils.fromPose2d(BiobuzzField.field().garden()), BiobuzzField.field().shoot()).constant(0);
        Path toFinalPark = line(BiobuzzField.field().shoot(), BiobuzzField.field().park()).constant(0);

        telemetry.addLine("Red Auto");
        telemetry.update();
        while (opModeInInit()) robot.update();
        if (isStopRequested()) return;

        LogUtil.init();
        long startedAt = System.currentTimeMillis();

        long t = System.currentTimeMillis();
        robot.shooter.setManual(false);

        robot.shooter.reqAim(true); // without this the shooter stays in TEST and never reads READY

        shoot(test, false); // fire preloads
        /*robot.intake.intake();
        follow(toGardenNew);
        robot.intake.stop();
        robot.intake.feed();
        follow(toShootPosAfterTip);
        shoot(test, false); // already at the shoot spot; `park` is the old y-offset path and leaves the field on red */
        follow(toFinalPark);

        //robot.intake.feed();
        //follow(toShoot);
        //shoot(park, false);
        //follow(park);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();

        telemetry.addData("done in (s)", "%.1f", (System.currentTimeMillis() - startedAt) / 1000.0);
        telemetry.addData("ended at", Globals.ROBOT_POSITION.toString());
        telemetry.update();
    }

    private void follow(Path path) {
        robot.drivetrain.follow(path);
        robot.waitWhile(robot.drivetrain::isBusy);
    }

    static final long READY_TIMEOUT_MS = 2500;

    private void shoot(Path shootMovement, boolean goToShootPos){
        if(goToShootPos) {
            robot.drivetrain.follow(shootMovement);
            robot.waitWhile(robot.drivetrain.follower::isBusy);
            robot.waitWhileWithTimeout(() -> robot.shooter.state != Shooter.State.READY, READY_TIMEOUT_MS);
        } else{
            robot.waitWhileWithTimeout(() -> robot.shooter.state != Shooter.State.READY, READY_TIMEOUT_MS);
        }

        // A target speed out of the flywheel's reach never reads READY, and AIMING ignores reqShoot: fire at the speed it reached.
        if (robot.shooter.state == Shooter.State.AIMING) robot.shooter.state = Shooter.State.READY;
        robot.shooter.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.reqStop(true);
        robot.shooter.reqAim(true);

    }
}
