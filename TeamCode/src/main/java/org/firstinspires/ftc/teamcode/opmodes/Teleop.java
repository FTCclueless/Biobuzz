package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
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
    /** gamepad1 dpad left toggles this fixed shot. Launch angle in degrees, like ShotTable2. */
    public static double PRESET_FLYWHEEL = 400;
    public static double PRESET_LAUNCH_DEG = 50;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap, true);

        robot.setStopChecker(this::isStopRequested);

        // Match the reference startup: automatic mode, with the blocker closed.
        robot.shooter.setManual(false);
        robot.shooter.setShooterBlocker(true);

        // Alliance and pose carry over from the last auto (nobody can touch the controllers in init).
        // No auto since the app started: alliance start pose of whatever side was last set.
        BiobuzzField.setSide(Globals.isRed ? BiobuzzField.Side.RED : BiobuzzField.Side.BLUE);
        Pose2d autoEnd = Globals.AUTO_ENDING_POSE;
        boolean fromAuto = autoEnd.x != 0 || autoEnd.y != 0 || autoEnd.heading != 0;
        robot.drivetrain.setPoseEstimate(fromAuto ? autoEnd.clone() : BiobuzzField.field().start());

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle outtake1 = new ButtonToggle();
        ButtonToggle lt1 = new ButtonToggle();
        ButtonToggle lt2 = new ButtonToggle();
        ButtonToggle rt1 = new ButtonToggle();
        ButtonToggle back1 = new ButtonToggle();
        ButtonToggle up1 = new ButtonToggle();
        ButtonToggle flowerToggle = new ButtonToggle();

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
        ButtonToggle up2 = new ButtonToggle();
        ButtonToggle down2 = new ButtonToggle();
        ButtonToggle left2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean confirmation = true;
        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            telemetry.addData("Alliance", Globals.isRed ? "RED" : "BLUE");
            telemetry.addData("Start pose from", fromAuto ? "last auto" : "alliance start (no auto run)");
            robot.sensors.update();
            robot.drivetrain.update();
            telemetry.addData("Pose", ROBOT_POSITION);
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            // Corner reset: gp2 RB or d-pad left. Evaluate both so each keeps its own edge state.
            boolean cornerRb = rb2.isClicked(gamepad2.right_bumper);
            boolean cornerDpad = left2.isClicked(gamepad2.dpad_left);
            if (cornerRb || cornerDpad) {
                robot.drivetrain.setPoseEstimate(nearestCorner(robot.drivetrain.getPoseEstimate()));
                gamepad2.rumble(200);
            }
            if (down2.isClicked(gamepad2.dpad_down)) { // pose to (0, 0, 0): field centre, facing +x
                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
                gamepad2.rumble(200);
            }
            robot.drivetrain.drive(gamepad1);

            // INTAKE: same gamepad1 controls as the reference Teleop.
            // LB flips between intake + transfer (full speed) and intake only (slower, no transfer). No off
            // stage: X (outtake) or a shot stops it. Not intaking forward (off, shot, outtake, reverse) starts at the first.
            if (lb1.isClicked(gamepad1.left_bumper)) {
                boolean intakingForward = robot.intake.state == Intake.State.INTAKE && !intakeReversed;
                boolean withTransfer = !intakingForward || !robot.intake.transferOnIntake;
                robot.intake.transferOnIntake = withTransfer;
                robot.intake.slowIntake = !withTransfer;
                robot.intake.reqIntake(true);
                intakeReversed = false;
                robot.intake.setRollerDirection(intakeReversed);
            }

            if (a1.isClicked(gamepad1.a && !gamepad1.start)) {
                intakeReversed = robot.intake.state == Intake.State.IDLE || !intakeReversed;
                robot.intake.transferOnIntake = false;
                robot.intake.slowIntake = false;
                robot.intake.reqIntake(true);
                robot.intake.setRollerDirection(intakeReversed);
            }

            // Hold the right stick down to creep balls up the transfer; tap repeatedly to jiggle a stuck one.
            robot.intake.nudgeTransfer = gamepad1.right_stick_button;

            if (flowerToggle.isClicked(gamepad1.dpad_down)) {
                robot.intake.toggleFlower();
            }

            // SHOOTER
            if (up2.isClicked(gamepad2.dpad_up)) { // turret kill: hold at 0 deg / press again to track
                robot.shooter.turretKilled = !robot.shooter.turretKilled;
                gamepad2.rumble(robot.shooter.turretKilled ? 600 : 150);
            }

            if (x2.isClicked(gamepad2.x)) {
                robot.shooter.pulseShooterBlocker();
            }

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
                // Manual flywheel/hood, turret still tracks the hive.
                robot.shooter.turretTrackInManual = true;
                robot.shooter.setManual(true);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            boolean shootClicked = rb1.isClicked(gamepad1.right_bumper);
            boolean shootReleased = rb1.isReleased(gamepad1.right_bumper);
            boolean closeClicked = b1.isClicked(gamepad1.b);
            boolean midClicked = y1.isClicked(gamepad1.y);
            boolean farClicked = x1.isClicked(gamepad1.dpad_right); // X is hold-to-outtake
            boolean pivotSwapClicked = lt1.isClicked(gamepad1.left_trigger > triggerThresh); // left trigger: swap pivot side
            boolean secondLeftTriggerClicked = lt2.isClicked(gamepad2.left_trigger > triggerThresh);
            boolean stopShooterClicked = rt1.isClicked(gamepad1.right_trigger > triggerThresh); // right trigger: flywheel off / stop aiming
            boolean closeHeld = b1.isHeld(gamepad1.b, 500);
            boolean midHeld = y1.isHeld(gamepad1.y, 500);
            boolean farHeld = x1.isHeld(gamepad1.dpad_right, 500);
            Shooter.forceUpdateVelBool = false;

            // Fixed-shot preset, from either mode. Shoot with right bumper as in manual; press again for off.
            if (up1.isClicked(gamepad1.dpad_left)) {
                boolean presetOn = robot.shooter.state == Shooter.State.TEST
                        && robot.shooter.flywheel.getTargetVelocity() == PRESET_FLYWHEEL;
                if (presetOn) {
                    robot.shooter.setShooter(Shooter.Dist.OFF);
                    flywheelOn = false;
                    atSpeedRumble = false;
                } else {
                    if (robot.shooter.state != Shooter.State.TEST) {
                        robot.shooter.reqShoot(false);
                        robot.shooter.reqAim(false);
                        robot.shooter.reqStop(false);
                        robot.shooter.setManual(true);
                    }
                    robot.shooter.setShooter(PRESET_FLYWHEEL, PRESET_LAUNCH_DEG);
                    flywheelOn = true;
                    atSpeedRumble = true;
                    confirmation = true;
                }
            }

            if (robot.shooter.state == Shooter.State.TEST) {

                if (stopShooterClicked || closeHeld
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

                if (gamepad2.right_trigger > triggerThresh) { // was RB, which is now the corner reset
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

                if (stopShooterClicked) {
                    robot.shooter.reqStop(true);
                }
            }

            // Each press switches the selected side (gp1 left trigger, gp2 left trigger); holding does not repeat.
            if (pivotSwapClicked || secondLeftTriggerClicked) {
                robot.intake.overridePivotSide = true;
                robot.intake.pivotSideFront = !robot.intake.pivotSideFront;
            }

            // Hold X to outtake (rollers and transfer reversed); release stops the intake.
            // Last before update, so it wins over a shot or intake command this loop.
            boolean outtakeReleased = outtake1.isReleased(gamepad1.x);
            if (gamepad1.x) robot.intake.eject();
            else if (outtakeReleased) robot.intake.stop();

            robot.update();

            telemetry.addData("shooter state", robot.shooter.state);
            robot.shooter.updateBallTarget(); // IDLE doesn't refresh it, and the half can change while driving
            telemetry.addData("alliance", Globals.isRed ? "RED" : "BLUE");
            telemetry.addData("hive distance (in)", "%.1f", Math.hypot(robot.shooter.ballTarget.x - ROBOT_POSITION.x,
                    robot.shooter.ballTarget.y - ROBOT_POSITION.y));
            telemetry.addData("table velocity / hood", "%.0f in/s / %.2f",
                    robot.shooter.minFlywheelVelocity, robot.shooter.targetHoodAngle);
            telemetry.addData("flywheelOn (manual)", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("turret", robot.shooter.turretKilled ? "KILLED (held at 0, gp2 dpad up)" : "tracking");
            telemetry.addData("turret (deg)", "%.1f / %.1f%s", Math.toDegrees(robot.shooter.turret.getAngle()),
                    Math.toDegrees(robot.shooter.turret.getTargetAngle()), robot.shooter.turret.atLimit() ? "  AT LIMIT" : "");
            telemetry.addData("flywheel position", robot.shooter.flywheel.getRawPosition());
            telemetry.addData("flywheel vel", robot.shooter.flywheel.getRawVelocity());
            telemetry.addData("intake state", robot.intake.state);
            telemetry.addData("intakeReversed", intakeReversed);
            // The current intake exposes intakeMotor rather than the reference's roller.
            telemetry.addData("intakePower", robot.intake.intakeMotor.getPower());
            Pose2d p = robot.drivetrain.getPoseEstimate();
            telemetry.addData("x", "%.2f", p.x);
            telemetry.addData("y", "%.2f", p.y);
            telemetry.addData("hdg", "%.2f", p.heading);
            telemetry.update();
        }

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    /** Robot squared into the nearest field corner: centre half a robot in from both walls, heading kept. */
    static Pose2d nearestCorner(Pose2d p) {
        // Along x the robot spans its length when it faces +-x, its width when it faces +-y.
        boolean facingX = Math.abs(Math.cos(p.heading)) >= Math.abs(Math.sin(p.heading));
        double halfX = (facingX ? Globals.ROBOT_LENGTH : Globals.ROBOT_WIDTH) / 2;
        double halfY = (facingX ? Globals.ROBOT_WIDTH : Globals.ROBOT_LENGTH) / 2;
        return new Pose2d(Math.copySign(72 - halfX, p.x), Math.copySign(72 - halfY, p.y), p.heading);
    }
}
