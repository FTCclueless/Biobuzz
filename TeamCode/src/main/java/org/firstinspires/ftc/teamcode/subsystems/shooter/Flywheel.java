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
    // Motor whose encoder input the flywheel encoder cable is plugged into. "" = shooter1's own port.
    // The drive motors' encoder inputs are free because Pedro localizes with the Pinpoint.
    // Read once at init, so restart the OpMode after changing it.
    public static String ENCODER_PORT = "leftFront";

    public static double TICKS_PER_MOTOR_REV = 28.0;

    public static double WHEEL_REVS_PER_MOTOR_REV = 20.0 / 30.0;

    public static double WHEEL_DIAMETER = 3.0;

    public static boolean MOTOR_REVERSED = false;

    public static boolean MOTOR_2_OPPOSED = true;

    public static double SHOT_DIP = 25.0;

    public static long SHOT_LOCKOUT_MS = 150;

    public static double kV = 0.019436;

    public static double kA = 0.002090;

    public static double kS = 1.5667;

    // LQR stiffness. Below ~30 the gain stops rising (one-step deadbeat limit) and loop delay makes
    // it overshoot; near top speed there are only ~1.5 V of headroom anyway, so softer costs little.
    public static double MAX_VELOCITY_ERROR = 100.0;

    public static double MAX_EFFORT = 8.0;

    // Integral trim, V per (in/s * s). The LQR gain alone is P-only and tops out near
    // kV*e^(-dt/tau)/(1-e^(-dt/tau)) (~0.09 V per in/s here), so a 1 V feedforward error
    // leaves a ~10 in/s offset whatever MAX_VELOCITY_ERROR is. This removes it.
    public static double kI = 0.1;
    /** Only integrate within this error (in/s), so spin-up and shot dips don't wind it up. */
    public static double I_BAND = 60.0;
    /** Most the integral may add or remove (V). */
    public static double I_MAX = 3.0;

    public static double CONTROL_DT = 0.02;
    // Command-to-measurement delay (s) the LQR predicts across: the hub's velocity reading and
    // the motor write each lag about a loop. Without it the near-deadbeat gain oscillates.
    public static double LATENCY = 0.04;

    public static final FlywheelFeedforward FF_TABLE = new FlywheelFeedforward();

    static {
        FF_TABLE.put(166.2, 4.736);
        FF_TABLE.put(259.7, 6.709);
        FF_TABLE.put(362.8, 8.629);
        FF_TABLE.put(463.6, 10.509);
        FF_TABLE.put(554.2, 12.361);
    }

    public static boolean USE_FF_TABLE = true;

    public static double MAX_SPIN_UP_ACCEL = 900.0;

    // Per-motor current limit, always on: current = (applied V - back-EMF) / R. Spikes only come
    // from accelerating (holding speed takes ~1 A), so this costs top speed nothing; spin-up is
    // still ~0.33 s to 500 in/s at 4 A. Lower it if the hubs brown out, raise it for faster recovery.
    public static double MAX_MOTOR_AMPS = 4.0;
    public static double MOTOR_RESISTANCE = 1.3; // ohms: 12 V / 9.2 A stall, bare goBILDA 6000 rpm

    public static double SPIN_UP_THRESH = 150.0;

    public static double VELOCITY_FILTER_THRESH = 60.0;
    public static double VELOCITY_FILTER_LOW = 0.05;
    public static double VELOCITY_FILTER_HIGH = 0.5;

    public static double AT_VEL_THRESH = 20.0;

    private final Robot robot;
    public final PriorityMotor flywheel;
    private final DcMotorEx motorRaw, encoder;

    private LQRController controller;
    private double builtWith_kV, builtWith_kA, builtWith_kS, builtWith_dt, builtWith_latency;
    private double builtWith_maxError, builtWith_maxEffort;

    private double targetVelocity = 0.0;
    private double rampedTarget = 0.0;
    private double filteredVelocity = 0.0;
    private double lastReference = 0.0;
    private double integral = 0.0;
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
        encoder = ENCODER_PORT.isEmpty() ? motorRaw : robot.hardwareMap.get(DcMotorEx.class, ENCODER_PORT);
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
                .latencyCompensation(LATENCY)
                .voltageCompensation(true);
        builtWith_kV = kV;
        builtWith_kA = kA;
        builtWith_kS = kS;
        builtWith_dt = CONTROL_DT;
        builtWith_latency = LATENCY;
        builtWith_maxError = MAX_VELOCITY_ERROR;
        builtWith_maxEffort = MAX_EFFORT;
    }

    private boolean constantsChanged() {
        return kV != builtWith_kV || kA != builtWith_kA || kS != builtWith_kS
                || CONTROL_DT != builtWith_dt
                || LATENCY != builtWith_latency
                || MAX_VELOCITY_ERROR != builtWith_maxError
                || MAX_EFFORT != builtWith_maxEffort;
    }

    public void update() {
        if (constantsChanged()) rebuildController();

        double actualVelocity = getRawVelocity();
        double alpha = Math.abs(actualVelocity - filteredVelocity) <= VELOCITY_FILTER_THRESH
                ? VELOCITY_FILTER_LOW : VELOCITY_FILTER_HIGH;
        filteredVelocity = filteredVelocity * (1 - alpha) + actualVelocity * alpha;

        // The first loop's loopTime spans since boot; a pause can be long too. Neither is a real step.
        double dt = robot.sensors.loopTime > 0 && robot.sensors.loopTime < 0.1
                ? robot.sensors.loopTime : CONTROL_DT;
        if (filteredVelocity < SPIN_UP_THRESH && targetVelocity > rampedTarget) {
            rampedTarget = Math.min(targetVelocity, rampedTarget + MAX_SPIN_UP_ACCEL * dt);
        } else {
            rampedTarget = targetVelocity;
        }

        double power;
        if (targetVelocity <= 1.0) {
            rampedTarget = 0;
            lastReference = 0;
            integral = 0;
            controller.reset(0, filteredVelocity);
            power = 0;
        } else {
            controller.setVelocityGoal(rampedTarget);
            // Inside the controller, so the latency predictor and anti-windup see the capped volts.
            controller.outputLimits(0.0, maxVolts(actualVelocity));
            power = controller.calculateVelocity(filteredVelocity, robot.sensors.getVoltage(),
                    feedforwardVolts(rampedTarget, dt) + integral);
            integral = integrate(integral, rampedTarget - filteredVelocity, dt, controller.isSaturated());
        }
        power = Math.max(0.0, Math.min(1.0, power));
        flywheel.setTargetPower(power);
        detectShot(actualVelocity);

        TelemetryUtil.packet.put("Flywheel : Power Applied", power * 100);
        TelemetryUtil.packet.put("Flywheel : Est Amps Per Motor",
                motorAmps(power * robot.sensors.getVoltage(), actualVelocity));
        TelemetryUtil.packet.put("Flywheel : Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Flywheel : Ramped Target", rampedTarget);
        TelemetryUtil.packet.put("Flywheel : Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Raw Velocity", actualVelocity);
        TelemetryUtil.packet.put("Flywheel : Encoder Ticks", getRawPosition());
        TelemetryUtil.packet.put("Flywheel : Shots", totalShots);
        TelemetryUtil.packet.put("Flywheel : Error", rampedTarget - filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Feedforward (V)", controller.lastFeedforward());
        TelemetryUtil.packet.put("Flywheel : Feedback (V)", controller.lastFeedback());
        TelemetryUtil.packet.put("Flywheel : Integral (V)", integral);
        TelemetryUtil.packet.put("Flywheel : Saturated", controller.isSaturated() ? 1 : 0);
        TelemetryUtil.packet.put("Flywheel : FF source", usingTable() ? 1 : 0);
        LogUtil.flywheelTarget.set(targetVelocity);
        LogUtil.flywheelVelocity.set(actualVelocity);
    }

    private double feedforwardVolts(double reference, double dt) {
        double refAccel = dt > 0 ? (reference - lastReference) / dt : 0;
        lastReference = reference;

        refAccel = Math.max(-MAX_SPIN_UP_ACCEL, Math.min(MAX_SPIN_UP_ACCEL, refAccel));

        return steadyVolts(reference) + kA * refAccel;
    }

    /** Volts that hold this speed: equal to the motors' back-EMF plus friction there. */
    private static double steadyVolts(double velocity) {
        return usingTable() ? FF_TABLE.volts(velocity) : kS + kV * velocity;
    }

    /** Back-EMF at this speed: the volts that hold it, minus the friction part (the volts that hold 0). */
    private static double backEmf(double velocity) { return steadyVolts(velocity) - steadyVolts(0); }

    /** Most volts that keep each motor under MAX_MOTOR_AMPS at this speed. */
    static double maxVolts(double velocity) {
        return Math.min(12.0, backEmf(velocity) + MAX_MOTOR_AMPS * MOTOR_RESISTANCE);
    }

    static double motorAmps(double volts, double velocity) {
        return (volts - backEmf(velocity)) / MOTOR_RESISTANCE;
    }

    static double integrate(double integral, double error, double dt, boolean saturated) {
        // Anti-windup: hold the integral while the output is clipped or the error is large.
        if (Math.abs(error) >= I_BAND || saturated) return integral;
        return Math.max(-I_MAX, Math.min(I_MAX, integral + kI * error * dt));
    }

    public static boolean usingTable() { return USE_FF_TABLE && !FF_TABLE.isEmpty(); }

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
        double motorRevsPerSec = encoder.getVelocity() / TICKS_PER_MOTOR_REV;
        return Math.abs(motorRevsPerSec * WHEEL_REVS_PER_MOTOR_REV * Math.PI * WHEEL_DIAMETER);
    }

    /** Encoder ticks, for checking the encoder counts at all. */
    public double getRawPosition() { return encoder.getCurrentPosition(); }

    public double getTargetVelocity() { return targetVelocity; }

    public double gain() { return controller.velocityGain(); }

    public boolean atVel() { return atVel(AT_VEL_THRESH); }

    public boolean atVel(double thresh) {
        return Math.abs(targetVelocity - filteredVelocity) <= thresh;
    }
}