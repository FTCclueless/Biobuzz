package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LQRController;
import org.firstinspires.ftc.teamcode.control.MechanismModel;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    public static double kV = 0.130;
    public static double kA = 0.0110;
    public static double kS = 0.090;
    public static double kG = 1.350;

    public static double TOLERANCE_POSITION = 0.30;
    public static double TOLERANCE_VELOCITY = 6.0;
    public static double MAX_EFFORT = 10.0;

    public static double MAX_VELOCITY = 40.0;
    public static double MAX_ACCEL = 120.0;

    public static double MIN_HEIGHT = 0.0;
    public static double MAX_HEIGHT = 30.0;
    public static double TICKS_PER_INCH = 25.1;

    public static double DT = 0.016;
    public static double LATENCY = 0.030;

    public static double REST_HEIGHT = 0.25;

    public enum State {
        RUNNING,
        RESTING,
        HOMING,
        MANUAL
    }

    public State state = State.RESTING;

    private final Robot robot;
    private final PriorityMotor motor;
    private LQRController controller;

    private double builtKV, builtKA, builtKS, builtKG, builtTolP, builtTolV, builtEffort;
    private double builtDT, builtLatency;

    private double tickOffset = 0;
    private double target = 0;
    private double manualPower = 0;
    private double lastPosition = 0, lastVelocity = 0;

    public Slides(Robot robot) {
        this.robot = robot;

        DcMotorEx[] raw = {robot.hardwareMap.get(DcMotorEx.class, "slides")};
        for (DcMotorEx m : raw) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motor = new PriorityMotor(raw, "slides", 3, 4, new double[] {1}, robot.sensors);
        motor.setMaxStaleness(0.08);
        robot.hardwareQueue.addDevice(motor);

        controller = buildController();
    }

    private void rebuildIfConfigChanged() {
        if (kV == builtKV && kA == builtKA && kS == builtKS && kG == builtKG
                && TOLERANCE_POSITION == builtTolP && TOLERANCE_VELOCITY == builtTolV
                && MAX_EFFORT == builtEffort
                && DT == builtDT && LATENCY == builtLatency) {
            return;
        }
        LQRController rebuilt = buildController();
        rebuilt.reset(lastPosition, lastVelocity);
        controller = rebuilt;
        if (state == State.RUNNING) {
            controller.setGoalFrom(lastPosition, target, MAX_VELOCITY, MAX_ACCEL);
        }
    }

    private LQRController buildController() {
        builtKV = kV; builtKA = kA; builtKS = kS; builtKG = kG;
        builtTolP = TOLERANCE_POSITION; builtTolV = TOLERANCE_VELOCITY; builtEffort = MAX_EFFORT;
        builtDT = DT; builtLatency = LATENCY;
        return new LQRController(MechanismModel.verticalSlide(kV, kA, kS, kG), DT)
                .tolerances(new double[]{TOLERANCE_POSITION, TOLERANCE_VELOCITY}, MAX_EFFORT)
                .atGoalTolerance(TOLERANCE_POSITION, TOLERANCE_VELOCITY)
                .latencyCompensation(LATENCY)
                .voltageCompensation(true);
    }

    public void setTarget(double inches) {
        double clamped = Utils.minMaxClip(inches, MIN_HEIGHT, MAX_HEIGHT);
        if (state == State.RUNNING && clamped == target) return;

        target = clamped;
        state = State.RUNNING;
        controller.setGoalFrom(getPosition(), target, MAX_VELOCITY, MAX_ACCEL);
    }

    public void setManualPower(double power) {
        state = State.MANUAL;
        manualPower = Utils.minMaxClip(power, -1.0, 1.0);
    }

    public void home() {
        state = State.HOMING;
    }

    public void rest() {
        target = 0;
        state = State.RESTING;
    }

    public double getPosition() {
        return (motor.motor[0].getCurrentPosition() - tickOffset) / TICKS_PER_INCH;
    }

    public double getVelocity() {
        return motor.getVelocity() / TICKS_PER_INCH;
    }

    public double getTarget() { return target; }

    public boolean atTarget() {
        return state == State.RESTING
                || (state == State.RUNNING && controller.atGoal(lastPosition, lastVelocity));
    }

    public boolean profileFinished() { return controller.profileFinished(); }

    public void zeroHere() { tickOffset = motor.motor[0].getCurrentPosition(); }

    public void update() {
        lastPosition = getPosition();
        lastVelocity = getVelocity();
        rebuildIfConfigChanged();
        double volts = robot.sensors.getVoltage();

        double power;
        switch (state) {
            case RUNNING:
                power = controller.calculate(lastPosition, lastVelocity, volts);

                if (target <= REST_HEIGHT && controller.profileFinished()
                        && lastPosition <= REST_HEIGHT) {
                    state = State.RESTING;
                    power = 0;
                }
                break;

            case HOMING:
                power = -HOMING_POWER;
                if (Math.abs(lastVelocity) < HOMING_STALL_VELOCITY) {
                    if (stallStart < 0) stallStart = System.nanoTime();
                    if ((System.nanoTime() - stallStart) / 1.0E9 >= HOMING_STALL_TIME) {
                        zeroHere();
                        controller.reset(0, 0);
                        target = 0;
                        stallStart = -1;
                        state = State.RESTING;
                        power = 0;
                    }
                } else {
                    stallStart = -1;
                }
                break;

            case MANUAL:
                power = manualPower;
                break;

            case RESTING:
            default:
                power = 0;
                break;
        }

        motor.setTargetPower(power);
        updateTelemetry(power);
    }

    public static double HOMING_POWER = 0.25;
    public static double HOMING_STALL_VELOCITY = 0.5;
    public static double HOMING_STALL_TIME = 0.25;

    private double stallStart = -1;

    private void updateTelemetry(double power) {
        TelemetryUtil.packet.put("Slides : state", state.toString());
        TelemetryUtil.packet.put("Slides : height", lastPosition);
        TelemetryUtil.packet.put("Slides : target", target);
        TelemetryUtil.packet.put("Slides : velocity", lastVelocity);
        TelemetryUtil.packet.put("Slides : power", power);
        TelemetryUtil.packet.put("Slides : reference", controller.referencePosition());
        TelemetryUtil.packet.put("Slides : feedforward V", controller.lastFeedforward());
        TelemetryUtil.packet.put("Slides : feedback V", controller.lastFeedback());
        TelemetryUtil.packet.put("Slides : saturated", controller.isSaturated());
        TelemetryUtil.packet.put("Slides : latency trust", controller.latencyTrust());
    }
}
