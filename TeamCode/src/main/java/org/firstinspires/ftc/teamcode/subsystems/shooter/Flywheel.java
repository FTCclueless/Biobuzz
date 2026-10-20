package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LQRController;
import org.firstinspires.ftc.teamcode.control.MechanismModel;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Flywheel {
    public static final String MOTOR_1 = "shooter1";
    public static final String MOTOR_2 = "shooter2";

    public static double TICKS_PER_MOTOR_REV = 28.0;

    public static double WHEEL_REVS_PER_MOTOR_REV = 20.0 / 30.0;

    public static double WHEEL_DIAMETER = 3.0;

    public static boolean MOTOR_REVERSED = false;

    public static boolean MOTOR_2_OPPOSED = true;

    public static double SHOT_DIP = 25.0;

    public static long SHOT_LOCKOUT_MS = 150;

    public static double kV = 0.0155;

    public static double kA = 0.0012;

    public static double kS = 0.35;

    public static double MAX_VELOCITY_ERROR = 8.0;

    public static double MAX_EFFORT = 8.0;

    public static double CONTROL_DT = 0.02;

    public static final FlywheelFeedforward FF_TABLE = new FlywheelFeedforward();

    static {
    }

    public static boolean USE_FF_TABLE = true;

    public static double MAX_SPIN_UP_ACCEL = 900.0;

    public static double SPIN_UP_THRESH = 150.0;

    public static double VELOCITY_FILTER_THRESH = 60.0;
    public static double VELOCITY_FILTER_LOW = 0.05;
    public static double VELOCITY_FILTER_HIGH = 0.5;

    public static double AT_VEL_THRESH = 20.0;

    private final Robot robot;
    public final PriorityMotor flywheel;
    private final DcMotorEx motorRaw;

    private LQRController controller;
    private double builtWith_kV, builtWith_kA, builtWith_kS, builtWith_dt;
    private double builtWith_maxError, builtWith_maxEffort;

    private double targetVelocity = 0.0;
    private double rampedTarget = 0.0;
    private double filteredVelocity = 0.0;
    private double lastReference = 0.0;
    private int pendingShots = 0;
    private int totalShots = 0;
    private long lastShotTime = 0;
    private boolean wasAtSpeed = false;

    public Flywheel(Robot robot) {
        this.robot = robot;

        motorRaw = robot.hardwareMap.get(DcMotorEx.class, MOTOR_1);
        DcMotorEx motor2 = robot.hardwareMap.get(DcMotorEx.class, MOTOR_2);
        for (DcMotorEx m : new DcMotorEx[]{motorRaw, motor2}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double dir = MOTOR_REVERSED ? -1 : 1;

        flywheel = new PriorityMotor(new DcMotorEx[]{motorRaw, motor2}, "flywheel", 3, 5,
                new double[]{dir, MOTOR_2_OPPOSED ? -dir : dir}, robot.sensors);

        robot.hardwareQueue.addDevice(flywheel);
        rebuildController();
    }

    private void rebuildController() {
        controller = new LQRController(MechanismModel.flywheel(kV, kA, kS), CONTROL_DT)
                .tolerances(new double[]{MAX_VELOCITY_ERROR}, MAX_EFFORT)

                .outputLimits(0.0, 12.0)
                .voltageCompensation(true);
        builtWith_kV = kV;
        builtWith_kA = kA;
        builtWith_kS = kS;
        builtWith_dt = CONTROL_DT;
        builtWith_maxError = MAX_VELOCITY_ERROR;
        builtWith_maxEffort = MAX_EFFORT;
    }

    private boolean constantsChanged() {
        return kV != builtWith_kV || kA != builtWith_kA || kS != builtWith_kS
                || CONTROL_DT != builtWith_dt
                || MAX_VELOCITY_ERROR != builtWith_maxError
                || MAX_EFFORT != builtWith_maxEffort;
    }

    public void update() {
        if (constantsChanged()) rebuildController();

        double actualVelocity = getRawVelocity();
        double alpha = Math.abs(actualVelocity - filteredVelocity) <= VELOCITY_FILTER_THRESH
                ? VELOCITY_FILTER_LOW : VELOCITY_FILTER_HIGH;
        filteredVelocity = filteredVelocity * (1 - alpha) + actualVelocity * alpha;

        double dt = robot.sensors.loopTime > 0 ? robot.sensors.loopTime : CONTROL_DT;
        if (filteredVelocity < SPIN_UP_THRESH && targetVelocity > rampedTarget) {
            rampedTarget = Math.min(targetVelocity, rampedTarget + MAX_SPIN_UP_ACCEL * dt);
        } else {
            rampedTarget = targetVelocity;
        }

        double power;
        if (targetVelocity <= 1.0) {
            rampedTarget = 0;
            lastReference = 0;
            controller.reset(0, filteredVelocity);
            power = 0;
        } else {
            controller.setVelocityGoal(rampedTarget);
            power = controller.calculateVelocity(filteredVelocity, robot.sensors.getVoltage(),
                    feedforwardVolts(rampedTarget, dt));
        }
        power = Math.max(0.0, Math.min(1.0, power));
        flywheel.setTargetPower(power);
        detectShot(actualVelocity);

        TelemetryUtil.packet.put("Flywheel : Power Applied", power * 100);
        TelemetryUtil.packet.put("Flywheel : Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Flywheel : Ramped Target", rampedTarget);
        TelemetryUtil.packet.put("Flywheel : Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Raw Velocity", actualVelocity);
        TelemetryUtil.packet.put("Flywheel : Shots", totalShots);
        TelemetryUtil.packet.put("Flywheel : Error", rampedTarget - filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Feedforward (V)", controller.lastFeedforward());
        TelemetryUtil.packet.put("Flywheel : Feedback (V)", controller.lastFeedback());
        TelemetryUtil.packet.put("Flywheel : Saturated", controller.isSaturated() ? 1 : 0);
        TelemetryUtil.packet.put("Flywheel : FF source", usingTable() ? 1 : 0);
        LogUtil.flywheelTarget.set(targetVelocity);
        LogUtil.flywheelVelocity.set(actualVelocity);
    }

    private double feedforwardVolts(double reference, double dt) {
        double refAccel = dt > 0 ? (reference - lastReference) / dt : 0;
        lastReference = reference;

        refAccel = Math.max(-MAX_SPIN_UP_ACCEL, Math.min(MAX_SPIN_UP_ACCEL, refAccel));

        if (!USE_FF_TABLE || FF_TABLE.isEmpty()) return Double.NaN;
        return FF_TABLE.volts(reference) + kA * refAccel;
    }

    public boolean usingTable() { return USE_FF_TABLE && !FF_TABLE.isEmpty(); }

    private void detectShot(double rawVelocity) {
        long now = System.currentTimeMillis();
        if (targetVelocity <= 1.0) {
            wasAtSpeed = false;
            return;
        }
        if (wasAtSpeed && targetVelocity - rawVelocity > SHOT_DIP
                && now - lastShotTime > SHOT_LOCKOUT_MS) {
            pendingShots++;
            totalShots++;
            lastShotTime = now;
            wasAtSpeed = false;
        } else if (Math.abs(targetVelocity - filteredVelocity) <= AT_VEL_THRESH) {
            wasAtSpeed = true;
        }
    }

    public int consumeShots() {
        int n = pendingShots;
        pendingShots = 0;
        return n;
    }

    public int getTotalShots() { return totalShots; }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public double getRawVelocity() {
        double motorRevsPerSec = motorRaw.getVelocity() / TICKS_PER_MOTOR_REV;
        return Math.abs(motorRevsPerSec * WHEEL_REVS_PER_MOTOR_REV * Math.PI * WHEEL_DIAMETER);
    }

    public double getTargetVelocity() { return targetVelocity; }

    public double gain() { return controller.velocityGain(); }

    public boolean atVel() { return atVel(AT_VEL_THRESH); }

    public boolean atVel(double thresh) {
        return Math.abs(targetVelocity - filteredVelocity) <= thresh;
    }
}
