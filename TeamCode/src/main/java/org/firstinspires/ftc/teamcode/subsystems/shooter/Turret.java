package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Turret {
    public static final String[] SERVO_NAMES = {"turret1", "turret2", "turret3"};
    public static final nPriorityServo.ServoType SERVO_TYPE =
            nPriorityServo.ServoType.AXON_MINI_EXTENDED;

    public static final boolean[] SERVO_REVERSED = {true, true, true};

    public static final String ENCODER_PORT = "";

    public static double ZERO_POSITION = 0.0963;

    public static double POSITION_PER_RADIAN = -SERVO_TYPE.positionPerRadian;

    public static boolean USE_ENCODER = true;

    public static double TICKS_PER_TURRET_REV = 8192 * 4;

    public static boolean ENCODER_REVERSED = true;

    public static long ZERO_SETTLE_MS = 1200;

    public static double MIN_ANGLE = Math.toRadians(-310);
    public static double MAX_ANGLE = Math.toRadians(50);
    public static double LIMIT_MARGIN = Math.toRadians(5);

    public static double TARGET_RATE_FILTER = 0.9;

    public static double LEAD_LOOPS = 1.0;

    public static double MAX_TARGET_RATE = 8.0;
    public static double IN_POSITION_TOLERANCE = Math.toRadians(1.5);

    public static double TRIM_GAIN = 2.0;

    public static double TRIM_WINDOW = Math.toRadians(6);

    public static double MAX_TRIM = Math.toRadians(4);

    public static double TRIM_MAX_TARGET_RATE = 0.5;

    private final Robot robot;
    public final nPriorityServo servos;
    private final DcMotorEx encoder;

    private boolean zeroed = false;
    private long settleStart = 0;
    private int zeroTicks = 0;
    private double encoderAngle = 0;

    private double requestedTarget = 0;
    private double target = 0;
    private double lastTarget = 0;
    private double targetRate = 0;
    private double trim = 0;
    private double command = 0;
    private Double manualPosition = null;

    public Turret(Robot robot) {
        this.robot = robot;
        Servo[] s = new Servo[SERVO_NAMES.length];
        for (int i = 0; i < s.length; i++) s[i] = robot.hardwareMap.get(Servo.class, SERVO_NAMES[i]);

        servos = new nPriorityServo(s, "turret", SERVO_TYPE, 0, 1, 0, SERVO_REVERSED, 5, 6);
        robot.hardwareQueue.addDevice(servos);
        encoder = ENCODER_PORT.isEmpty() ? null : robot.hardwareMap.tryGet(DcMotorEx.class, ENCODER_PORT);
    }

    public boolean hasEncoder() { return encoder != null; }

    private boolean encoderActive() { return encoder != null && USE_ENCODER; }

    private void readEncoder() {
        if (encoder == null) return;
        double sign = ENCODER_REVERSED ? -1 : 1;
        encoderAngle = sign * (encoder.getCurrentPosition() - zeroTicks)
                / TICKS_PER_TURRET_REV * 2 * Math.PI;
    }

    public void update() {
        readEncoder();
        double dt = robot.sensors.loopTime > 0 ? robot.sensors.loopTime : 0.02;

        double rawRate = (target - lastTarget) / dt;
        targetRate = targetRate * (1 - TARGET_RATE_FILTER) + rawRate * TARGET_RATE_FILTER;
        targetRate = Math.max(-MAX_TARGET_RATE, Math.min(MAX_TARGET_RATE, targetRate));
        lastTarget = target;

        updateZero();

        if (zeroed && encoderActive() && TRIM_GAIN > 0
                && Math.abs(targetRate) < TRIM_MAX_TARGET_RATE
                && Math.abs(target - encoderAngle) < TRIM_WINDOW) {
            trim += TRIM_GAIN * (target - encoderAngle) * dt;
            trim = Math.max(-MAX_TRIM, Math.min(MAX_TRIM, trim));
        }

        double lead = targetRate * dt * LEAD_LOOPS;
        command = clampToWindow(target + lead + trim);

        double pos = manualPosition != null ? manualPosition
                : ZERO_POSITION + command * POSITION_PER_RADIAN;
        servos.setTargetPos(pos);

        TelemetryUtil.packet.put("Turret : Angle (deg)", Math.toDegrees(getAngle()));
        TelemetryUtil.packet.put("Turret : Target (deg)", Math.toDegrees(target));
        TelemetryUtil.packet.put("Turret : Error (deg)", Math.toDegrees(target - getAngle()));
        TelemetryUtil.packet.put("Turret : Servo estimate (deg)", Math.toDegrees(servoEstimate()));
        TelemetryUtil.packet.put("Turret : Trim (deg)", Math.toDegrees(trim));
        TelemetryUtil.packet.put("Turret : Zeroed", zeroed ? 1 : 0);
        LogUtil.turretAngle.set(getAngle());
        LogUtil.turretTarget.set(target);
    }

    private void updateZero() {
        if (zeroed || manualPosition != null) return;
        boolean holdingZero = target == 0 && servos.inPosition(Math.toRadians(0.5));
        if (!holdingZero) {
            settleStart = 0;
            return;
        }
        long now = System.currentTimeMillis();
        if (settleStart == 0) settleStart = now;
        if (now - settleStart >= ZERO_SETTLE_MS) zeroEncoderHere();
    }

    public void zeroEncoderHere() {
        if (encoder == null) return;
        zeroTicks = encoder.getCurrentPosition();
        encoderAngle = 0;
        trim = 0;
        zeroed = true;
    }

    public void rezero() {
        zeroed = false;
        settleStart = 0;
    }

    private double servoEstimate() {
        double servoPos = servos.getCurrentAngle() * SERVO_TYPE.positionPerRadian;
        return (servoPos - ZERO_POSITION) / POSITION_PER_RADIAN;
    }

    static double servoAngleAt(double position) {
        return (position - ZERO_POSITION) / POSITION_PER_RADIAN;
    }

    static double lowLimit() {
        double servoLow = Math.min(servoAngleAt(0), servoAngleAt(1));
        return Math.max(MIN_ANGLE, servoLow) + LIMIT_MARGIN;
    }

    static double highLimit() {
        double servoHigh = Math.max(servoAngleAt(0), servoAngleAt(1));
        return Math.min(MAX_ANGLE, servoHigh) - LIMIT_MARGIN;
    }

    private static double clampToWindow(double a) {
        return Math.max(lowLimit(), Math.min(highLimit(), a));
    }

    public void setTargetAngle(double robotRelativeAngle) {
        requestedTarget = robotRelativeAngle;
        target = chooseReachable(robotRelativeAngle, getAngle());
    }

    static double chooseReachable(double desired, double current) {
        double lo = lowLimit();
        double hi = highLimit();
        double twoPi = 2 * Math.PI;

        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        double base = desired + twoPi * Math.round((current - desired) / twoPi);
        for (int k = -1; k <= 1; k++) {
            double c = base + k * twoPi;
            if (c < lo || c > hi) continue;
            double d = Math.abs(c - current);
            if (d < bestDist) {
                bestDist = d;
                best = c;
            }
        }
        if (!Double.isNaN(best)) return best;

        double toLo = Math.abs(normalize(desired - lo));
        double toHi = Math.abs(normalize(desired - hi));
        return toLo < toHi ? lo : hi;
    }

    private static double normalize(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    public static double clipAngle(double robotRelativeAngle) {
        return chooseReachable(robotRelativeAngle, 0);
    }

    public double getAngle() {
        return zeroed && encoderActive() ? encoderAngle : servoEstimate();
    }

    public double getEncoderAngle() { return encoderAngle; }

    public double getTargetAngle() { return target; }

    public double getTrim() { return trim; }

    public boolean isZeroed() { return zeroed; }

    public boolean inPosition() { return inPosition(IN_POSITION_TOLERANCE); }

    public boolean inPosition(double tolerance) {
        return Math.abs(target - getAngle()) <= tolerance;
    }

    public boolean atLimit() {
        return Math.abs(normalize(requestedTarget - target)) > Math.toRadians(0.5);
    }

    public void setManualPosition(Double position) { manualPosition = position; }
}
