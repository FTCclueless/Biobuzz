package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
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

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drivetrain.setPoseEstimate(startPose);

        Trajectory out = DriveConstants.pathBuilder()
                .start(0, 0, 0)
                .end(SIZE, 0, 0)
                .velocities(0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
                .endHeading(0)
                .build();

        Trajectory around = DriveConstants.pathBuilder()
                .start(SIZE, 0, 0)
                .waypoint(SIZE, SIZE * 0.5, 3.0)
                .end(SIZE, SIZE, 0)
                .velocities(0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
                .build();

        Trajectory home = DriveConstants.pathBuilder()
                .start(SIZE, SIZE, 0)
                .waypoint(SIZE * 0.5, SIZE, 3.0)
                .end(0, 0, 0, 0)
                .velocities(0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
                .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
                .endHeading(0)
                .markerBeforeEnd(12, "almost home", () -> {
                })
                .build();

        telemetry.addLine("Example Auto");
        telemetry.addData("total distance (in)", "%.0f",
                out.length() + around.length() + home.length());
        telemetry.addData("planned time (s)", "%.1f",
                out.duration() + around.duration() + home.duration());
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }
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

    private void follow(Trajectory trajectory) {
        robot.drivetrain.followTrajectory(trajectory);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }
}
