package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

/**
 * Pedro's Mecanum, but wheel powers go through our HardwareQueue instead of straight to the
 * motors, so drive writes are ranked and budgeted with the rest of the robot's writes.
 * Pedro still owns the motor directions and zero-power behavior (set on the same DcMotorEx).
 */
@Config
public class QueuedMecanum extends Mecanum {
    public static double BASE_PRIORITY = 4, PRIORITY_SCALE = 5; // same as the old drivetrain
    // Over budget, the queue does one ranked write per loop; critical makes every changed wheel write.
    public static boolean CRITICAL = false;

    private final PriorityMotor[] queued = new PriorityMotor[4]; // Pedro order: FL, FR, BL, BR

    public QueuedMecanum(HardwareMap h, MecanumConfig c, HardwareQueue queue, Sensors sensors) {
        super(h, c);
        String[] names = {c.frontLeftName.get(), c.frontRightName.get(), c.backLeftName.get(), c.backRightName.get()};
        for (int i = 0; i < 4; i++) {
            queued[i] = new PriorityMotor(h.get(DcMotorEx.class, names[i]), names[i],
                    BASE_PRIORITY, PRIORITY_SCALE, sensors);
            queued[i].setCritical(CRITICAL);
        }
        queue.addDevices(queued);
    }

    /** Same normalization as Mecanum.applyDrive; only the write is different. */
    @Override
    public void applyDrive(DrivePowers powers) {
        double[] w = computeWheelPowersUnnormalized(powers);
        double max = 1;
        for (double v : w) max = Math.max(max, Math.abs(v));
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = w[i] / max;
            queued[i].setTargetPower(wheelPowers[i]);
        }
    }

    @Override
    public void stop(boolean brake) {
        setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = 0;
            queued[i].setTargetPower(0);
        }
    }
}
