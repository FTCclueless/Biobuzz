package org.firstinspires.ftc.teamcode.utils.priority;

import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.CRServo;

public class PriorityCRServo extends PriorityDevice {
    public enum ServoType {
        // Radians/s is speed
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(1 / Math.toRadians(305), 5.3403953024772129),
        AXON_MAX(0.1775562245447108, 6.5830247235911042),
        AXON_MICRO(0.1775562245447108, 6.5830247235911042),  // TODO need to tune
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

    public CRServo[] servo;
    double power = 0;
    double lastPower = 0;
    boolean reversed[];
    private double angle = 0;
    public final ServoType servoType;

    public PriorityCRServo(CRServo servo, String name, ServoType servoType, boolean[] reversed, double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.servo = new CRServo[]{servo};
        this.reversed = reversed;
        this.servoType = servoType;
    }

    public PriorityCRServo(CRServo[] servos, String name, ServoType servoType, boolean[] reversed, double basePriority, double priorityScale) { //one of the servos must be reversed prior to use
        super(basePriority, priorityScale, name);
        this.servo = servos;
        this.reversed = reversed;
        this.servoType = servoType;
    }

    public void setTargetPower(double power) {
        this.power = power;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (power-lastPower == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }
        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }
        return basePriority + Math.abs(power-lastPower) + (System.nanoTime()-lastUpdateTime)/1.0E6 * priorityScale;
    }

    @Override
    public void update() {
        for(int i = 0; i < servo.length; i++){
            servo[i].setPower(Utils.clamp(power, -1.0, 1.0) * (reversed[i] ? -1.0 : 1.0));
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}