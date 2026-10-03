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

/**
 * Flywheel velocity control by LQR.
 *
 * A flywheel is the easiest mechanism to model: one state (surface speed, in/s) and one input
 * (volts). The model is kA * dv/dt = V - kV * v - kS, so the LQR reduces to a single optimal
 * gain on velocity error, sitting on top of a kV/kA/kS feedforward that already knows roughly
 * what voltage this speed needs. The feedback only has to cover the difference, which is why
 * this recovers from a shot faster than a PID that has to rebuild an integral term.
 *
 * All three model constants come from the Flywheel Characterizer opmode. Run it before trusting
 * anything here -- with wrong constants the feedforward fights the feedback.
 */
@Config
public class Flywheel {
    // ------------------------------------------------------------------ model (Characterizer)
    /** Volts per in/s of steady speed. Dominates the feedforward. */
    public static double kV = 0.0155;
    /** Volts per in/s^2 of acceleration. Sets how fast it can spin up. */
    public static double kA = 0.0012;
    /** Volts needed just to overcome friction and start moving. */
    public static double kS = 0.35;

    // ------------------------------------------------------------------ LQR tuning
    /**
     * Velocity error, in/s, you are willing to accept before the controller should spend
     * MAX_EFFORT volts fixing it. Smaller = stiffer and more aggressive.
     */
    public static double MAX_VELOCITY_ERROR = 8.0;
    /** Volts the controller may spend on feedback. Raise for a snappier recovery. */
    public static double MAX_EFFORT = 8.0;
    /** Control period, seconds. Should match the robot loop time. */
    public static double CONTROL_DT = 0.02;

    // ------------------------------------------------------------------ feedforward table
    /**
     * Measured volts-per-speed, from the Flywheel Characterizer. When it has entries they are
     * used instead of the linear kV/kS feedforward, because real flywheels bend away from a
     * straight line at high speed. kA still comes from the model for the acceleration term,
     * and the LQR feedback is unchanged.
     *
     * Empty = fall back to the linear model, which is a perfectly good starting point.
     */
    public static final FlywheelFeedforward FF_TABLE = new FlywheelFeedforward();

    static {
        // Paste the "=== paste into Flywheel FF_TABLE ===" lines from Flywheel Characterizer.
        // Example shape (do not use these numbers, they are not from your robot):
        // FF_TABLE.put(310.0, 4.85);
        // FF_TABLE.put(430.0, 6.60);
    }

    /** Set false to force the linear kV/kS feedforward even when the table has entries. */
    public static boolean USE_FF_TABLE = true;

    // ------------------------------------------------------------------ shaping
    /** Cap on how fast the target ramps, in/s per second. Stops belt skip on spin-up. */
    public static double MAX_SPIN_UP_ACCEL = 900.0;
    /** Speed below which the ramp applies; above it the wheel is already moving. */
    public static double SPIN_UP_THRESH = 150.0;
    /** Velocity readings jumping more than this are treated as noise and filtered harder. */
    public static double VELOCITY_FILTER_THRESH = 60.0;
    public static double VELOCITY_FILTER_LOW = 0.05;
    public static double VELOCITY_FILTER_HIGH = 0.5;
    /** Error, in/s, within which atVel() reports ready to shoot. */
    public static double AT_VEL_THRESH = 20.0;

    private final Robot robot;
    public final PriorityMotor flywheel;

    private LQRController controller;
    private double builtWith_kV, builtWith_kA, builtWith_kS, builtWith_dt;
    private double builtWith_maxError, builtWith_maxEffort;

    private double targetVelocity = 0.0;
    private double rampedTarget = 0.0;
    private double filteredVelocity = 0.0;
    private double lastReference = 0.0;

    public Flywheel(Robot robot) {
        this.robot = robot;

        DcMotorEx ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        ms1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ms2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2}, "flywheel", 3, 5,
                new double[]{1, -1}, robot.sensors);

        robot.hardwareQueue.addDevice(flywheel);
        rebuildController();
    }

    /**
     * Solving the Riccati equation is too slow to do every loop, so the controller is rebuilt
     * only when a constant actually changes -- which in practice means when someone edits one
     * in Dashboard.
     */
    private void rebuildController() {
        controller = new LQRController(MechanismModel.flywheel(kV, kA, kS), CONTROL_DT)
                .tolerances(new double[]{MAX_VELOCITY_ERROR}, MAX_EFFORT)
                // Never command negative volts: braking a flywheel wastes the energy that the
                // next shot needs, and the motor cannot pull the wheel backwards usefully.
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

        double actualVelocity = robot.sensors.getFlywheelVelocity();
        double alpha = Math.abs(actualVelocity - filteredVelocity) <= VELOCITY_FILTER_THRESH
                ? VELOCITY_FILTER_LOW : VELOCITY_FILTER_HIGH;
        filteredVelocity = filteredVelocity * (1 - alpha) + actualVelocity * alpha;

        // Ramp the reference rather than the output. The controller still sees a reachable
        // target, so the feedforward stays honest and the feedback never winds up chasing a
        // speed the wheel cannot reach yet.
        double dt = robot.sensors.loopTime > 0 ? robot.sensors.loopTime : CONTROL_DT;
        if (filteredVelocity < SPIN_UP_THRESH && targetVelocity > rampedTarget) {
            rampedTarget = Math.min(targetVelocity, rampedTarget + MAX_SPIN_UP_ACCEL * dt);
        } else {
            rampedTarget = targetVelocity;
        }

        double power;
        if (targetVelocity <= 1.0) {
            // Coast. A stopped flywheel needs no holding voltage.
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

        TelemetryUtil.packet.put("Flywheel : Power Applied", power * 100);
        TelemetryUtil.packet.put("Flywheel : Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Flywheel : Ramped Target", rampedTarget);
        TelemetryUtil.packet.put("Flywheel : Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Error", rampedTarget - filteredVelocity);
        TelemetryUtil.packet.put("Flywheel : Feedforward (V)", controller.lastFeedforward());
        TelemetryUtil.packet.put("Flywheel : Feedback (V)", controller.lastFeedback());
        TelemetryUtil.packet.put("Flywheel : Saturated", controller.isSaturated() ? 1 : 0);
        TelemetryUtil.packet.put("Flywheel : FF source", usingTable() ? 1 : 0); // 1 = table
        LogUtil.flywheelTarget.set(targetVelocity);
    }

    /**
     * Volts to hold the reference speed: the measured table when it has data, otherwise the
     * linear model. While the reference is still ramping, the kA term is added so the wheel
     * is pushed toward the speed it is about to be asked for, not just the one it has now.
     */
    private double feedforwardVolts(double reference, double dt) {
        double refAccel = dt > 0 ? (reference - lastReference) / dt : 0;
        lastReference = reference;
        // A ramp step is a clean number; a loop-time glitch is not. Cap it at the ramp rate.
        refAccel = Math.max(-MAX_SPIN_UP_ACCEL, Math.min(MAX_SPIN_UP_ACCEL, refAccel));

        if (!USE_FF_TABLE || FF_TABLE.isEmpty()) return Double.NaN; // model feedforward
        return FF_TABLE.volts(reference) + kA * refAccel;
    }

    /** True when the table is in use, for telemetry. */
    public boolean usingTable() { return USE_FF_TABLE && !FF_TABLE.isEmpty(); }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    /** The single optimal velocity-error gain, volts per in/s. For telemetry and sanity checks. */
    public double gain() { return controller.velocityGain(); }

    public boolean atVel() { return atVel(AT_VEL_THRESH); }

    public boolean atVel(double thresh) {
        return Math.abs(targetVelocity - filteredVelocity) <= thresh;
    }
}
