package org.firstinspires.ftc.teamcode.utils.priority;

import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

public class nPriorityServo extends PriorityDevice {
    public enum ServoType {
        // Radians/s is speed
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(1 / Math.toRadians(305), 5.3403953024772129),
        AXON_MINI_EXTENDED(1 / Math.toRadians(322), 5), // Experimentally found
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

    public final Servo[] servos;
    private final ServoType type;
    public final double minPos;
    public final double maxPos;
    public final double basePos;
    private double currentAngle = 0, targetAngle = 0, power = 1.0, currentIntermediateTargetAngle = 0;
    protected final boolean[] reversed;
    private long lastLoopTime = Globals.LOOP_START;
    private boolean first = true; // Priority servo has a problem when the servos won't get set at the start if theyre set to 0
    private boolean forceUpdate = false;
    public double maxPower = 1.0;

    /**
     * Basic initializer
     *
     * @param servos If servos are connected in parallel add them all here
     * @param name Name of device for HardwareQueue lookup
     * @param type Type of servo type
     * @param minPos Minimum pose that it can possibly move to
     * @param maxPos Maximum pose that it can possibly move to
     * @param basePos Pose that is set t0 "0"
     * @param reversed Which servos in the servo array are reversed or not
     * @param basePriority BP
     * @param priorityScale PS
     */
    public nPriorityServo(Servo[] servos, String name, ServoType type, double minPos, double maxPos, double basePos, boolean[] reversed, double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.servos = servos;
        this.type = type;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.basePos = basePos;
        this.reversed = reversed;
        this.currentAngle = convertPosToAngle(basePos);
        if (type == ServoType.HITEC) { // I actually dislike this servo so much
            servos[0].setPosition(1.0);
            servos[0].setPosition(0.0);
            servos[0].setPosition(basePos);
        }

        /*

        for (Servo s : servos) {
            if (s instanceof ServoImplEx) {
                ((ServoImplEx) s).setPwmRange(new PwmControl.PwmRange(500, 2500));
            }
        }

         */


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

    public boolean inPosition() {
        //Log.e("ERIC LOG", "inPosition is " + (Math.abs(targetAngle-currentAngle) < Math.toRadians(0.01)) + "");
        return Math.abs(targetAngle-currentAngle) < Math.toRadians(0.1);
    }

    public boolean inPosition(double thresh) {
        return Math.abs(targetAngle - currentAngle) < thresh;
    }

    public void setTargetAngle(double angle) { setTargetAngle(angle, 1); }

    public void setTargetAngle(double angle, double power) {
        if (!Double.isNaN(angle)) this.targetAngle = Utils.clamp(angle, convertPosToAngle(minPos), convertPosToAngle(maxPos));
        this.power = Utils.clamp(power, 0, this.maxPower);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetPos(double pos) { this.setTargetPos(pos, 1); }

    public void setTargetPos(double pos, double power) {
        if (!Double.isNaN(pos)) this.targetAngle = convertPosToAngle(Utils.clamp(pos, minPos, maxPos));
        this.power = Utils.clamp(power, 0, this.maxPower);
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
        //Log.e("Priority Servo Log", name + " is moving with power " + power);

        forceUpdate = false;

        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = (currentTime - lastUpdateTime) / 1.0E9;

        double error = targetAngle - currentAngle;
        //Log.e("TTTTTTTTa", timeSinceLastUpdate + " is the time since last update");
        //Log.e("TTTTTTTTa", error + " is the error");
        double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);
        //Log.e("TTTTTTTTa", deltaAngle + " delta ang");
        //Log.e("TTTTTTTTa", power + " power");

        currentIntermediateTargetAngle += deltaAngle;

        // Clamp
        if (Math.abs(deltaAngle) > Math.abs(error) || power == 1)
            currentIntermediateTargetAngle = targetAngle;
        //Log.e("ooga", "booga");

        // Update servos
        for (int i = 0; i < servos.length; i++) {
            double pos = 0;
            if (!reversed[i]) {
                pos = convertAngleToPos(currentIntermediateTargetAngle);
            } else {
                pos = 1 - convertAngleToPos(currentIntermediateTargetAngle);
            }
            if (type == ServoType.HITEC && pos <= 0.07) {
                // I kid you not this must happen
                servos[i].setPosition(0.1);
                servos[i].setPosition(0.07);
            }

            servos[i].setPosition(pos);
            // servos[i].setPosition(Utils.clamp(pos + (i % 2 == 1 ? 0.01 : 0), 0, 1));
            //Log.i("SLCI", "Set position of " + name + i + " to position " + pos + " current angle is " + currentAngle);
        }

        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        // STUPID STUPID HACK I HATE YOU
        if (first) {
            if (!(Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER)) {
                update();
            }
            first = false;
        }

        // Update the servo internal values
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastLoopTime) / 1.0E9;

        // We actually use this to pretty much just get direction
        double error = targetAngle - currentAngle;
        //Log.i("SLCI", "currentIntermediateTargetAngle is " + currentIntermediateTargetAngle + " for " + name);
        //Log.i("SLCI", "currentAngle is " + currentAngle + " for " + name);
        //Log.i("SLCI", "targetAngle is " + targetAngle + " for " + name);

        // How much the servo has moved from the start of the loop to now
        double deltaAngle = loopTime * type.speed * Math.signum(error) * power;

//        Log.e("adding " + this.name + "deltaAngle" , deltaAngle + "");
//        Log.e(this.name + "'s current angle" , currentAngle + "");
//        Log.e(this.name + "_loopTime" , loopTime + "");
//        Log.e(this.name + "_type.speed" , type.speed + "");
//        Log.e(this.name + "_error" , error + "");
//        Log.e(this.name + "_power" , power + "");
//        Log.e(this.name + "_targetAngle" , targetAngle + "");
//        Log.e(this.name + "_currentAngle" , currentAngle + "");
//        Log.e(this.name + "_currentIntermediateTargetAngle" , currentIntermediateTargetAngle + "");

        currentAngle += deltaAngle;
/*
        TelemetryUtil.packet.put("servo currentAngle " + name, currentAngle);
        TelemetryUtil.packet.put("servo error " + name, error);
        TelemetryUtil.packet.put("servo deltaAngle " + name, deltaAngle);
        TelemetryUtil.packet.put("servo targetAngle " + name, targetAngle);
        TelemetryUtil.packet.put("servo inPosition " + name, inPosition());
*/
        // Clamp
        //Log.i("SLCI", deltaAngle + " is delta angle for servo " + name);
        //Log.i("SLCI", error + " is error for servo " + name);
        if (Math.abs(error) < Math.abs(deltaAngle)) {
            //    Log.i("SLCI", "Gotcha! " + name);
            currentAngle = targetAngle;
        }

        lastLoopTime = currentTime;

        if (isUpdated)
            return 0;

        // Dawg what the hell??
        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        // Ong trust this function.
        double priority = (((currentAngle - targetAngle) != 0) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;

        // Yuh that means it just updated. Dont even touch that thing
        if (priority == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        if (forceUpdate) priority += 1000;

        return priority;
    }
}