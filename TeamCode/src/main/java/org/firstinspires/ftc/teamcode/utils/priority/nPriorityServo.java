package org.firstinspires.ftc.teamcode.utils.priority;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class nPriorityServo extends PriorityDevice {
    public enum ServoType {
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(1 / Math.toRadians(305), 5.3403953024772129),
        AXON_MINI_EXTENDED(1 / Math.toRadians(322), 5),
        AXON_MAX(0.1775562245447108, 6.5830247235911042),
        AXON_MICRO(0.1775562245447108, 6.5830247235911042),
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12),
        HITEC(0.2966648, 5.2);

        public final double positionPerRadian;
        public final double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public final Servo[] servos;
    private final ServoType type;
    public final double minPos;
    public final double maxPos;
    public final double basePos;
    private double currentAngle = 0, targetAngle = 0, power = 1.0, currentIntermediateTargetAngle = 0;
    protected final boolean[] reversed;
    public static final double CALL_LENGTH_MILLIS = 1.2;

    private double lastWrittenAngle = Double.NaN;
    private boolean first = true;
    private boolean forceUpdate = false;
    public double maxPower = 1.0;

    public nPriorityServo(Servo[] servos, String name, ServoType type, double minPos, double maxPos, double basePos, boolean[] reversed, double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.servos = servos;
        this.type = type;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.basePos = basePos;
        this.reversed = reversed;
        this.currentAngle = convertPosToAngle(basePos);
        this.actuatorCount = servos.length;
        this.callLengthMillis = CALL_LENGTH_MILLIS;
        if (type == ServoType.HITEC) {
            servos[0].setPosition(1.0);
            servos[0].setPosition(0.0);
            servos[0].setPosition(basePos);
        }
    }

    private double convertPosToAngle(double pos) {
        pos -= basePos;
        pos /= type.positionPerRadian;
        return pos;
    }

    private double convertAngleToPos(double ang) {
        ang *= type.positionPerRadian;
        ang += basePos;
        return ang;
    }

    private double minAngle() {
        return Math.min(convertPosToAngle(minPos), convertPosToAngle(maxPos));
    }

    private double maxAngle() {
        return Math.max(convertPosToAngle(minPos), convertPosToAngle(maxPos));
    }

    public boolean inPosition() {
        return Math.abs(targetAngle-currentAngle) < Math.toRadians(0.1);
    }

    public boolean inPosition(double thresh) {
        return Math.abs(targetAngle - currentAngle) < thresh;
    }

    public void setTargetAngle(double angle) { setTargetAngle(angle, 1); }

    public void setTargetAngle(double angle, double power) {
        if (!Double.isNaN(angle)) this.targetAngle = Utils.minMaxClip(angle, convertPosToAngle(minPos), convertPosToAngle(maxPos));
        this.power = Utils.minMaxClip(power, 0, this.maxPower);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetPos(double pos) { this.setTargetPos(pos, 1); }

    public void setTargetPos(double pos, double power) {
        if (!Double.isNaN(pos)) this.targetAngle = convertPosToAngle(Utils.minMaxClip(pos, minPos, maxPos));
        this.power = Utils.minMaxClip(power, 0, this.maxPower);
    }

    public double getTargetPos() {
        return convertAngleToPos(targetAngle);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public void setForceUpdate() { forceUpdate = true; }

    @Override
    protected void update() {
        forceUpdate = false;

        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = (currentTime - lastUpdateTime) / 1.0E9;

        double error = targetAngle - currentAngle;
        double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);

        currentIntermediateTargetAngle += deltaAngle;

        if (Math.abs(deltaAngle) > Math.abs(error) || power == 1
                || Math.abs(targetAngle - currentIntermediateTargetAngle) < 1e-9)
            currentIntermediateTargetAngle = targetAngle;

        currentIntermediateTargetAngle =
                Utils.minMaxClip(currentIntermediateTargetAngle, minAngle(), maxAngle());

        for (int i = 0; i < servos.length; i++) {
            double pos = convertAngleToPos(currentIntermediateTargetAngle);
            if (reversed[i]) pos = 1 - pos;

            pos = Utils.minMaxClip(pos, 0.0, 1.0);

            if (type == ServoType.HITEC && pos <= 0.07) {
                servos[i].setPosition(0.1);
                servos[i].setPosition(0.07);
            }

            servos[i].setPosition(pos);
        }

        lastWrittenAngle = currentIntermediateTargetAngle;
        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    @Override
    protected void onAdvance(double dt) {
        if (first) {
            if (!(Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER)) {
                update();
            }
            first = false;
        }

        double error = targetAngle - currentAngle;
        double deltaAngle = dt * type.speed * Math.signum(error) * power;
        currentAngle += deltaAngle;

        if (Math.abs(error) < Math.abs(deltaAngle)) {
            currentAngle = targetAngle;
        }
    }

    @Override
    protected double commandedValue() { return convertAngleToPos(targetAngle); }

    @Override
    protected boolean hasPendingWrite() {
        if (forceUpdate) return true;
        return Double.isNaN(lastWrittenAngle) || lastWrittenAngle != targetAngle;
    }

    @Override
    protected double error() {
        if (Double.isNaN(lastWrittenAngle)) return 1.0;
        return Math.abs(convertAngleToPos(targetAngle) - convertAngleToPos(lastWrittenAngle));
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (isUpdated) return 0;
        if (!hasPendingWrite()) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }
        if (timeRemaining * 1000.0 <= costMillis()) {
            return 0;
        }
        if (forceUpdate) return Double.MAX_VALUE;
        return rank();
    }
}
