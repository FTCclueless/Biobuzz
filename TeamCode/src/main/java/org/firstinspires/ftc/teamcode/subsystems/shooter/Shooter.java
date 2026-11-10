package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Shooter {
    public enum State {
        IDLE,
        AIMING,
        READY,
        SHOOT,
        TEST
    } public State state = State.IDLE;

    private final Robot robot;
    public final Flywheel flywheel;
    public final Turret turret;
    public final nPriorityServo hood, flywheelBlocker;
    private final ShotTable2 shooterTable;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false;
    public boolean turretTrackInManual = false;

    public double targetHoodAngle = 0.0;
    public static double hoodSweep = Math.toRadians(26.43);
    public static double hoodGearRatio = 20.0 / 40.0 * 254.0 / 35.0;

    public static double latchBlockAngle = 1, latchOpenAngle = 0.2;

    public Vector3 ballTarget;
    public Vector3 targetOverride = null;
    public double minFlywheelVelocity = 0.0;

    public static double SOTMThreshold = 10;
    public static double flywheelExitReadyThresh = 50;

    public static boolean autoShootIfInZone = false;

    public static boolean AUTO_STOP_WHEN_EMPTY = true;
    public static long EMPTY_GRACE_MS = 300;
    private long emptySince = -1;
    public static boolean forceUpdateVelBool = false;
    public static double forceUpdateVel;

    public Shooter(Robot robot) {
        this.robot = robot;

        this.flywheel = new Flywheel(robot);
        this.turret = new Turret(robot);

        this.shooterTable = new ShotTable2();

        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.03, 0.33, 0.03,
            new boolean[] {false},
            6, 7
        );

        flywheelBlocker = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
            "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
            0, 0.7, 0.1,
            new boolean[] {false},
            2, 2
        );

        robot.hardwareQueue.addDevices(hood, flywheelBlocker);

        updateBallTarget();
    }

    public void update() {
        switch (state) {
            case IDLE: {
                stopRequest = false;
                flywheel.setTargetVelocity(forceUpdateVelBool ? forceUpdateVel : Dist.CLOSE.flywheelVel);
                setHoodAngle(0.0);
                setShooterBlocker(true);
                turret.setTargetAngle(0);

                if (aimRequest) {
                    aimRequest = false;
                    state = State.AIMING;
                }
                break;
            }
            case AIMING: {
                aimRequest = false;

                setShooterBlocker(true);
                //TelemetryUtil.packet.put("Aim: aimLauncherV8", "before");
                //boolean aimResult = aimLauncherV8();
                predictGoal2AxisInterpolate();
                boolean turretResult = turret.inPosition();
                //TelemetryUtil.packet.put("Aim: aimResult", aimResult);
                TelemetryUtil.packet.put("Aim: turretResult", turretResult);
                TelemetryUtil.packet.put("Aim: hood.inPosition", hood.inPosition());
                TelemetryUtil.packet.put("Aim: atVel", atVel());
                // An empty or out-of-range shot table gives 0 in/s, which would count as "at speed".
                if (turretResult && this.atVel() && hood.inPosition() && minFlywheelVelocity > 0) {
                    state = State.READY;
                }
                flywheel.setTargetVelocity(forceUpdateVelBool ? forceUpdateVel : minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    stopRequest = false;
                    aimRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                }
                break;
            }
            case READY: {
                if (!this.atVel(flywheelExitReadyThresh) && Globals.RUNMODE == RunMode.TELEOP) {
                    state = State.AIMING;
                }

                setShooterBlocker(true);

                predictGoal2AxisInterpolate();

                flywheel.setTargetVelocity(forceUpdateVelBool ? forceUpdateVel : minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (shootRequest /* && (isRobotInZone(0,0,-72,72,-72,-72) || isRobotInZone(48,0,72,24,72,-24)) */) {
                    setShooterBlocker(false);
                    if (flywheelBlocker.inPosition()) {
                        state = State.SHOOT;
                        robot.intake.reqShoot(true);
                    }
                }

                if (autoShootIfInZone && isRobotInZone() && Math.hypot(ROBOT_GLOBAL_VELOCITY.x, ROBOT_GLOBAL_VELOCITY.y) < SOTMThreshold) {
                    setShooterBlocker(false);
                    if (flywheelBlocker.inPosition()) {
                        state = State.SHOOT;
                        robot.intake.reqShoot(true);
                    }
                }

                if (stopRequest) {
                    stopRequest = false;
                    shootRequest = false;
                    flywheel.setTargetVelocity(Dist.CLOSE.flywheelVel);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);

                    state = State.IDLE;
                }

                break;
            }
            case SHOOT: {
                shootRequest = false;
                predictGoal2AxisInterpolate();
                setShooterBlocker(false);
                flywheel.setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                boolean empty = false;
                if (AUTO_STOP_WHEN_EMPTY && robot.intake.getBallCount() == 0) {
                    long now = System.currentTimeMillis();
                    if (emptySince < 0) emptySince = now;
                    empty = now - emptySince >= EMPTY_GRACE_MS;
                } else {
                    emptySince = -1;
                }

                if (empty) {
                    emptySince = -1;
                    state = State.IDLE;
                    flywheel.setTargetVelocity(Dist.CLOSE.flywheelVel);
                    robot.intake.reqOff(true);
                } else if (stopRequest) {
                    stopRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    flywheel.setTargetVelocity(Dist.CLOSE.flywheelVel);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                } else if (autoShootIfInZone && !isRobotInZone()) {
                    state = State.AIMING;
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            }
            case TEST: {
                updateBallTarget();

                if (turretTrackInManual) {
                    double turretAngle = Math.atan2(ballTarget.getY() - ROBOT_POSITION.y, ballTarget.getX() - ROBOT_POSITION.x);
                    turret.setTargetAngle(turretAngle - ROBOT_POSITION.heading);
                }
                break;
            }
        }

        flywheel.update();
        turret.update();

        updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Shooter : state", this.state);
        TelemetryUtil.packet.put("Shooter : Balltarget", ballTarget.toString());
        TelemetryUtil.packet.put("Shooter : goal distance", Math.hypot(ROBOT_POSITION.x - robot.shooter.ballTarget.x, ROBOT_POSITION.y - robot.shooter.ballTarget.y));

        LogUtil.shooterState.set(this.state.toString());
        LogUtil.hoodAngle.set(hood.getTargetAngle());
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setStroke("#808080");
        canvas.setStrokeWidth(1);
        canvas.strokeLine(ballTarget.x, ballTarget.y, ROBOT_POSITION.x, ROBOT_POSITION.y);
        canvas.setStroke(Globals.isRed ? "#ff0000" : "#0000ff");
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(ballTarget.x, ballTarget.y, 2.5);
    }

    public void reqShoot (boolean req) { shootRequest = req; }

    public void reqAim (boolean req) { aimRequest = req; }

    public void reqStop (boolean req) { stopRequest = req; }

    public void setManual(boolean on) {
        if (on) {
            state = State.TEST;
            turret.setTargetAngle(0);
            setShooter(Dist.OFF);
        } else {
            state = State.IDLE;
        }
    }

    public void setHoodAngle(double target_angle) { hood.setTargetAngle(target_angle); }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : latchOpenAngle);}

    public void updateBallTarget() {
        ballTarget = targetOverride != null ? targetOverride
                : BiobuzzField.nearestHiveCell(Globals.isRed, ROBOT_POSITION);
    }

    public boolean atVel() { return flywheel.atVel(); }
    public boolean atVel(double thresh) { return flywheel.atVel(thresh); }

    // further separation :)
    // bootleg LM1 strat being used in LM2 & LM3 code

    public enum Dist {
        CLOSE(0.5, 450),
        AUTO_POSITION(0.6, 460),
        MID(1.35, 520),
        FAR(1.3, 610),
        OFF(0.0, 0.0);

        private final double hoodAngle, flywheelVel;

        Dist(double hoodAngle, double flywheelVel) {
            this.hoodAngle = hoodAngle;
            this.flywheelVel = flywheelVel;
        }
    }

    public void setShooter(Dist mode) {
        flywheel.setTargetVelocity(mode.flywheelVel);
        setHoodAngle(targetHoodAngle = mode.hoodAngle);
    }

    public static boolean isRobotInZone() {
        return isRobotInZone(-2,0,-72,70,-72,-70) || isRobotInZone(50,0,72,22,72,-22);
    }

    public static boolean isRobotInZone(double x1, double y1, double x2, double y2, double x3, double y3) {
        //getting robot pose info
        double theta = ROBOT_POSITION.heading;
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);
        double l = Globals.ROBOT_LENGTH;
        double w = Globals.ROBOT_WIDTH;
        double rx = ROBOT_POSITION.x;
        double ry = ROBOT_POSITION.y;

        //Four corners of robot - relative to robot
        double[][] corners = {
                {l/2, w/2}, {l/2, -w/2}, {-l/2, w/2}, {-l/2, -w/2}, {0, 0}
        };

        //Calculate triangle denom
        double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);

        //Transform the corners of robot to global coordinates and check weights
        for (double[] c : corners) {
            // Rotation + Translation of each corner
            double px = rx + (c[0] * cosT - c[1] * sinT);
            double py = ry + (c[0] * sinT + c[1] * cosT);

            // Barycentric weight calculation
            double w1 = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom;
            double w2 = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom;
            double w3 = 1.0 - w1 - w2;

            //none of the triangle areas are negative meaning robot is inside the triangle
            if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
                return true;
            }
        }

        return false;
    }

    public void predictGoal2AxisInterpolate() {
        updateBallTarget();
        double currFlywheelVel = flywheel.getFilteredVelocity();

        double initialDist = Math.hypot(ballTarget.x - ROBOT_POSITION.x, ballTarget.y - ROBOT_POSITION.y);
        double virtualX = ballTarget.x;
        double virtualY = ballTarget.y;
        minFlywheelVelocity = shooterTable.getFlywheelForDistance(initialDist);
        targetHoodAngle = shooterTable.getLaunchAngleForDistanceAndFlywheel(initialDist, currFlywheelVel);

        if (Math.hypot(ROBOT_GLOBAL_VELOCITY.x, ROBOT_GLOBAL_VELOCITY.y) >= SOTMThreshold && currFlywheelVel >= 300) {
            // Launch angle is from horizontal, so the ball's horizontal speed is v*cos.
            double time = initialDist / (currFlywheelVel / 2 * Math.cos(targetHoodAngle));

            virtualX = ballTarget.x - (ROBOT_GLOBAL_VELOCITY.x * time + 0.5 * ROBOT_GLOBAL_ACCELERATION.x * time * time);
            virtualY = ballTarget.y - (ROBOT_GLOBAL_VELOCITY.y * time + 0.5 * ROBOT_GLOBAL_ACCELERATION.y * time * time);

            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            canvas.setStroke(Globals.isRed ? "#ff4000" : "#0040ff");
            canvas.setStrokeWidth(2);
            canvas.strokeCircle(virtualX, virtualY, 2.5);

            double virtualDist = Math.hypot(virtualX - ROBOT_POSITION.x, virtualY - ROBOT_POSITION.y);
            minFlywheelVelocity = shooterTable.getFlywheelForDistance(virtualDist);
            targetHoodAngle = shooterTable.getLaunchAngleForDistanceAndFlywheel(virtualDist, currFlywheelVel);
        }

        TelemetryUtil.packet.put("Aim : target hood launch angle (deg)", Math.toDegrees(targetHoodAngle));
        targetHoodAngle = Utils.minMaxClip((targetHoodAngle - hoodSweep) * hoodGearRatio, 0, 1.6);
        double virtualTurretAngle = Math.atan2(virtualY - ROBOT_POSITION.y, virtualX - ROBOT_POSITION.x);
        turret.setTargetAngle(virtualTurretAngle - ROBOT_POSITION.heading);
    }
}
