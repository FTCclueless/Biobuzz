package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name="Base Auto", group = "Auto")
public class BiobuzzAutoBase extends LinearOpMode
{
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(BiobuzzField.field().start());

        while (opModeInInit()) {
            robot.update();
        }

        // example of field agnostic
        //robot.drivetrain.goToPoint(BiobuzzField.field().garden(), 1.0);
    }

    private void follow(Trajectory trajectory) {
        robot.drivetrain.followTrajectory(trajectory);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }
}
