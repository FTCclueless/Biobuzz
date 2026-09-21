package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.nMergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@Config
public class Drivetrain {
    public enum State {
        PID_TO_POINT,
        BRAKE,
        WAIT,
        DRIVE,
        FOLLOW_TRAJECTORY,
        IDLE
    }
    public State state = State.IDLE;

    private PathFollower trajectoryFollower;
    private MecanumKinematics kinematics;
    private Pose2d trajectoryEndPose;
    private double[][] trajectoryDrawPoints;

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private final List<PriorityMotor> motors;

    public Vision vision;
    public Localizer localizer;
    public nMergeLocalizer nMergeLocalizer;
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;

    public static double targetHeading = 135;
    public static double headingLockDeadzone = 2.5;
    private boolean wasLocking = false;

    public Drivetrain(Robot robot, Vision vision) { this(robot.hardwareMap, robot.sensors, robot.hardwareQueue, vision); }

    public Drivetrain(HardwareMap hardwareMap, Sensors sensors, HardwareQueue hardwareQueue, Vision vision) {
        this.vision = vision;
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

        leftFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront", 4, 5,
            1.0, sensors
        );
        leftRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear", 4, 5,
            1.0, sensors
        );
        rightRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear", 4, 5,
            1.0, sensors
        );
        rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront", 4, 5,
            1.0, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (PriorityMotor motor : motors) {
            motor.setCritical(true).setMaxStaleness(0.10);
        }

        configureMotors();
        setMinPowersToOvercomeFriction(1.0);

        localizer = new Localizer (sensors, this, "#ff0000", "#ffffff");
        nMergeLocalizer = new nMergeLocalizer (hardwareMap, sensors, this, "#0000ff", "#ff00ff");
    }

    public void configureMotors() {
        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            hardwareQueue.addDevice(motor);
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    double[] minPowersToOvercomeFriction = {
        0.3, 0.3, 0.3, 0.3
    };

    public void setMinPowersToOvercomeFriction(double scalar) {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0] * scalar);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1] * scalar);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2] * scalar);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3] * scalar);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.145);
        }
    }

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        leftRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightFront.setMinimumPowerToOvercomeStaticFriction(0.0);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0);
        }
    }

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
        nMergeLocalizer.setPoseEstimate(pose2d);
        LogUtil.drivePositionReset = true;
    }

    public Pose2d getPoseEstimate() {
        return ROBOT_POSITION;
    }

    private Pose2d targetPoint = new Pose2d (0, 0, 0);
    public static PID xPID = new PID (0.2, 0.0, 0.007);
    public static PID yPID = new PID (0.2, 0.0, 0.007);
    public static PID turnPID = new PID (0.4, 0.0, 0.002);
    public static PID hPID = new PID (0.53, 0.0, 0.0);
    public static double turnKStatic = 0.15;
    public static double xThresh = 1.5, yThresh = 1.5, hThresh = Math.toRadians(2.5), waypointThresh = 3.0;
    public static double xError = 0.0, yError = 0.0, hError = 0.0;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        switch (state) {
            case PID_TO_POINT:
                calculateErrors();
                PIDF();
                if (atPoint()) {
                    state = isWaypoint ? State.WAIT : State.BRAKE;
                    xPID.resetIntegral();
                    yPID.resetIntegral();
                    turnPID.resetIntegral();
                }
                break;
            case BRAKE:
                stopAllMotors();
                state = State.WAIT;
                break;
            case WAIT:
                calculateErrors();
                if (!atPoint()) {
                    state = State.PID_TO_POINT;
                }
                break;
            case FOLLOW_TRAJECTORY:
                PathFollower.Command cmd =
                        trajectoryFollower.update(ROBOT_POSITION, sensors.loopTime);

                setMotorPowers(cmd.powers[0], cmd.powers[2], cmd.powers[3], cmd.powers[1]);

                TelemetryUtil.packet.put("Path : s", cmd.s);
                TelemetryUtil.packet.put("Path : contour error", cmd.contourError);
                TelemetryUtil.packet.put("Path : lag error", cmd.lagError);
                TelemetryUtil.packet.put("Path : saturation", cmd.saturation);

                if (trajectoryFollower.isFinished() || trajectoryFollower.hasFaulted()) {
                    if (trajectoryFollower.hasFaulted()) {
                        Log.e("Drivetrain", "path follower fault: " + trajectoryFollower.fault());
                    }
                    stopAllMotors();
                    if (trajectoryEndPose != null) this.targetPoint = trajectoryEndPose;
                    state = State.WAIT;
                }
                break;
            case DRIVE:
                break;
            case IDLE:
                break;
        }

        updateTelemetry();
    }

    private void calculateErrors() {
        double deltaX = (targetPoint.x - ROBOT_POSITION.x);
        double deltaY = (targetPoint.y - ROBOT_POSITION.y);

        xError = Math.cos(ROBOT_POSITION.heading)*deltaX + Math.sin(ROBOT_POSITION.heading)*deltaY;
        yError = -Math.sin(ROBOT_POSITION.heading)*deltaX + Math.cos(ROBOT_POSITION.heading)*deltaY;
        hError = AngleUtil.clipAngle(targetPoint.heading - ROBOT_POSITION.heading);
    }

    double fwd, strafe, h;

    private void PIDF() {
        fwd = xPID.update(xError, -maxPower, maxPower);
        strafe = yPID.update(yError, -maxPower, maxPower);
        h = turnPID.update(hError, -maxPower, maxPower);
        if (hError > hThresh) h += turnKStatic;
        if (hError < -hThresh) h -= turnKStatic;

        setMinPowersToOvercomeFriction(1.0);

        setMoveVector(new Vector2(fwd, strafe), h);
    }

    private boolean atPoint() {
        if (isWaypoint) return Math.abs(xError) < waypointThresh && Math.abs(yError) < waypointThresh;
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < hThresh;
    }

    private double maxPower = 1.0;
    private boolean isWaypoint = false;
    public static double KIN_TRACK_WIDTH = 14.0, KIN_WHEEL_BASE = 13.0, MAX_WHEEL_SPEED = 66.0;

    public static double PATH_CONTOUR_GAIN = 3.2, PATH_LAG_GAIN = 0.9;
    public static double PATH_HEADING_GAIN = 4.0, PATH_ACCEL_LEAD = 0.03;

    public void followTrajectory(Trajectory trajectory) {
        if (kinematics == null) {
            kinematics = new MecanumKinematics(KIN_TRACK_WIDTH, KIN_WHEEL_BASE, MAX_WHEEL_SPEED);
        }
        trajectoryEndPose = trajectory.poseAt(trajectory.length());
        this.isWaypoint = false;

        trajectory.resetMarkers();
        trajectoryDrawPoints = DashboardUtil.samplePath(trajectory.path());
        trajectoryFollower = new PathFollower(trajectory, kinematics)
                .contourGain(PATH_CONTOUR_GAIN)
                .lagGain(PATH_LAG_GAIN)
                .headingGain(PATH_HEADING_GAIN)
                .accelLead(PATH_ACCEL_LEAD);
        state = State.FOLLOW_TRAJECTORY;
    }

    public PathFollower trajectoryFollower() { return trajectoryFollower; }

    public void goToPoint(Pose2d targetPoint, double maxPower) { goToPoint(targetPoint, maxPower, false); };
    public void goToPoint(Pose2d targetPoint, double maxPower, boolean isWaypoint) {
        Pose2d lastTargetPoint = this.targetPoint;
        this.targetPoint = targetPoint;
        this.maxPower = maxPower;
        this.isWaypoint = isWaypoint;

        if (lastTargetPoint.x != targetPoint.x || lastTargetPoint.y != targetPoint.y || lastTargetPoint.heading != targetPoint.heading || state == State.DRIVE) {
            xPID.resetIntegral();
            yPID.resetIntegral();
            turnPID.resetIntegral();
            state = State.PID_TO_POINT;
        }
    }

    public void setMoveVector(Vector2 moveVector, double turn) {
        double[] powers = {
                moveVector.x - turn - moveVector.y,
                moveVector.x - turn + moveVector.y,
                moveVector.x + turn - moveVector.y,
                moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);

        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

        TelemetryUtil.packet.put("Drivetrain : moveVector x", moveVector.x);
        TelemetryUtil.packet.put("Drivetrain : moveVector y", moveVector.y);
        TelemetryUtil.packet.put("Drivetrain : moveVector turn", turn);
    }

    public static double smoothPowerK = 0.5;
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPowerSmooth(lf, smoothPowerK);
        leftRear.setTargetPowerSmooth(lr, smoothPowerK);
        rightRear.setTargetPowerSmooth(rr, smoothPowerK);
        rightFront.setTargetPowerSmooth(rf, smoothPowerK);
    }

    public void stopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setTargetPower(0);
        }
    }

    public void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }

    public void drive(Gamepad gamepad) { drive(gamepad, false); }
    public void drive(Gamepad gamepad, boolean lockHeading) {
        resetMinPowersToOvercomeFriction();
        state = State.DRIVE;

        double forward = smoothControls(-1 * gamepad.left_stick_y);
        double strafe = smoothControls(-1 * gamepad.left_stick_x);

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05) {
            drive.mul(0);
        }

        double turn;

        if (lockHeading) {
            if (!wasLocking) {
                turnPID.resetIntegral();
            }
            wasLocking = true;

            double error = AngleUtil.clipAngle(Math.toRadians(targetHeading) * (Globals.isRed ? 1 : -1) - ROBOT_POSITION.heading);
            if (Math.abs(error) < Math.toRadians(headingLockDeadzone)) {
                turn = 0;
                turnPID.resetIntegral();
            } else {
                turn = turnPID.update(error, -maxPower, maxPower) + turnKStatic * Math.signum(error);
            }
        } else {
            wasLocking = false;
            turn = smoothControls(-gamepad.right_stick_x);
        }

        setMoveVector(drive, turn);
    }

    public double smoothControls(double value) {
        return 0.7 * Math.tan(0.96 * value);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Drivetrain : state", state);

        TelemetryUtil.packet.put("Drivetrain : PID xError", xError);
        TelemetryUtil.packet.put("Drivetrain : PID yError", yError);
        TelemetryUtil.packet.put("Drivetrain : PID hError", hError);

        TelemetryUtil.packet.put("Drivetrain : PID xPow", fwd);
        TelemetryUtil.packet.put("Drivetrain : PID yPow", strafe);
        TelemetryUtil.packet.put("Drivetrain : PID hPow", h);

        LogUtil.driveState.set(state.toString());

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        if (state == State.FOLLOW_TRAJECTORY && trajectoryFollower != null) {
            Trajectory traj = trajectoryFollower.trajectory();
            DashboardUtil.drawSampledPath(canvas, trajectoryDrawPoints);

            DashboardUtil.drawRobot(canvas, traj.poseAt(trajectoryFollower.arcLength()), "#8000ff");
            LogUtil.drivePath.set(traj.path().toString());
        } else {
            DashboardUtil.drawRobot(canvas, targetPoint, isWaypoint ? "#c040ff" : "#8000ff");
            LogUtil.drivePath.set(String.format(Locale.US, "%.3f %.3f %.3f", targetPoint.x, targetPoint.y, targetPoint.heading));
        }
    }
}
