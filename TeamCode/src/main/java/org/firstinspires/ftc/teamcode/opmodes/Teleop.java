package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Locale;

@Config
@TeleOp(name = "Teleop Main")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap, true);

        robot.setStopChecker(this::isStopRequested);

        // Match the reference startup: automatic mode, with the blocker closed.
        robot.shooter.setManual(false);
        robot.shooter.setShooterBlocker(true);

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));


        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle lt1 = new ButtonToggle();
        ButtonToggle lt2 = new ButtonToggle();
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

        boolean intakeReversed = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean confirmation = true;
        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            robot.sensors.update();
            robot.drivetrain.update();
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }

        while (!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);

            // INTAKE: same gamepad1 controls as the reference Teleop.
            if (lb1.isClicked(gamepad1.left_bumper)) {
                if (robot.intake.state == Intake.State.IDLE) robot.intake.reqIntake(true);
                else robot.intake.reqOff(true);
                intakeReversed = false;
                robot.intake.setRollerDirection(intakeReversed);
            }

            if (a1.isClicked(gamepad1.a && !gamepad1.start)) {
                intakeReversed = robot.intake.state == Intake.State.IDLE || !intakeReversed;
                robot.intake.reqIntake(true);
                robot.intake.setRollerDirection(intakeReversed);
            }

            // SHOOTER

            if (a2.isClicked(gamepad2.a)) {
                robot.intake.reqOff(true);
                robot.shooter.setShooterBlocker(true);
                robot.shooter.reqShoot(false);
                robot.shooter.reqAim(false);
                robot.shooter.reqStop(false);
                flywheelOn = false;
                atSpeedRumble = false;
                confirmation = true;
                robot.shooter.setManual(true);
                robot.shooter.turretTrackInManual = false;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            if (b2.isClicked(gamepad2.b && !gamepad2.start)) {
                robot.intake.reqOff(true);
                robot.shooter.setShooterBlocker(true);
                robot.shooter.reqShoot(false);
                robot.shooter.reqAim(false);
                robot.shooter.reqStop(false);
                flywheelOn = false;
                atSpeedRumble = false;
                confirmation = true;
                robot.shooter.setManual(false);
                robot.shooter.setShooter(Shooter.Dist.OFF);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            if (y2.isClicked(gamepad2.y)) {
                robot.intake.reqOff(true);
                robot.shooter.setShooterBlocker(true);
                robot.shooter.reqShoot(false);
                robot.shooter.reqAim(false);
                robot.shooter.reqStop(false);
                flywheelOn = false;
                atSpeedRumble = false;
                confirmation = true;
                // Reference tracking-mode button retained; turret hardware is disabled on master.
                robot.shooter.turretTrackInManual = true;
                robot.shooter.setManual(true);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            boolean shootClicked = rb1.isClicked(gamepad1.right_bumper);
            boolean shootReleased = rb1.isReleased(gamepad1.right_bumper);
            boolean closeClicked = b1.isClicked(gamepad1.b);
            boolean midClicked = y1.isClicked(gamepad1.y);
            boolean farClicked = x1.isClicked(gamepad1.x);
            boolean leftTriggerClicked = lt1.isClicked(gamepad1.left_trigger > triggerThresh);
            boolean secondLeftTriggerClicked = lt2.isClicked(gamepad2.left_trigger > triggerThresh);
            boolean rightTriggerClicked = rt1.isClicked(gamepad1.right_trigger > triggerThresh);
            boolean closeHeld = b1.isHeld(gamepad1.b, 500);
            boolean midHeld = y1.isHeld(gamepad1.y, 500);
            boolean farHeld = x1.isHeld(gamepad1.x, 500);
            Shooter.forceUpdateVelBool = false;

            if (robot.shooter.state == Shooter.State.TEST) {

                if (rightTriggerClicked || closeHeld
                        || midHeld
                        || farHeld) { // Off
                    flywheelOn = false;
                    robot.shooter.setShooter(Shooter.Dist.OFF);
                    atSpeedRumble = false;
                } else if (closeClicked) { // Close
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.CLOSE);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (midClicked) { // Middle
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.MID);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (farClicked) { // Far
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.FAR);
                    atSpeedRumble = true;
                    confirmation = true;
                }

                if (atSpeedRumble && confirmation) {
                    confirmation = false;
                } else if (atSpeedRumble && robot.shooter.atVel()) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    atSpeedRumble = false;
                }

                if (gamepad1.right_bumper) {
                    robot.shooter.setShooterBlocker(false);
                    robot.intake.reqShoot(true);
                } else if (shootReleased) {
                    robot.shooter.setShooterBlocker(true);
                    robot.intake.reqOff(true);
                }
            } else {

                if (gamepad2.right_bumper) {
                    Shooter.forceUpdateVelBool = true;
                    Shooter.forceUpdateVel = 580;
                } else if (gamepad2.left_bumper) {
                    Shooter.forceUpdateVelBool = true;
                    Shooter.forceUpdateVel = 400;
                } else {
                    Shooter.forceUpdateVelBool = false;
                }

                if (midClicked) {
                    robot.shooter.reqAim(true);
                }

                if (shootClicked) {
                    if (robot.shooter.state == Shooter.State.READY) robot.shooter.reqShoot(true);
                } else if (shootReleased) {
                    // Cancel even if the blocker was still opening when released.
                    robot.shooter.reqShoot(false);
                    robot.shooter.setShooterBlocker(true);
                    if (robot.shooter.state == Shooter.State.SHOOT) {
                        robot.shooter.reqStop(true);
                        robot.shooter.reqAim(true);
                    }
                }

                if (robot.shooter.state == Shooter.State.READY) {
                    if (confirmation) {
                        gamepad1.rumble(150);
                        gamepad2.rumble(150);
                        confirmation = false;
                    }
                } else {
                    confirmation = true;
                }

                if (rightTriggerClicked) {
                    robot.shooter.reqStop(true);
                }
            }

            // Each press switches the selected side; holding a trigger does not repeat.
            if (leftTriggerClicked || secondLeftTriggerClicked) {
                robot.intake.overridePivotSide = true;
                robot.intake.pivotSideFront = !robot.intake.pivotSideFront;
            }

            robot.update();

            telemetry.addData("shooter state", robot.shooter.state);
            telemetry.addData("flywheelOn (manual)", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("turret tracking available", false);
            telemetry.addData("flywheel position", robot.shooter.flywheel.getRawPosition());
            telemetry.addData("flywheel vel", robot.shooter.flywheel.getRawVelocity());
            telemetry.addData("intake state", robot.intake.state);
            telemetry.addData("intakeReversed", intakeReversed);
            // The current intake exposes intakeMotor rather than the reference's roller.
            telemetry.addData("intakePower", robot.intake.intakeMotor.getPower());
            telemetry.update();
        }

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
