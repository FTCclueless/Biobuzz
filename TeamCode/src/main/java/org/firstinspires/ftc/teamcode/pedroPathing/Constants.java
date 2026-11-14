package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.RunMode;

public class Constants {
    public static MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftRear");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.xPodOffset.set(5.793267717);
        c.yPodOffset.set(0.629921);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.brake.set(Controller.pid(0.2, 0, 0));
        c.headingFeedback.set(Controller.pid(2, 0, 0.01));
        c.linearBrakeCoefficients.set(Matrix.diag(0.0788, 0.0788));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00191035, 0.00191035));
        c.maxAchievableForwardVelocity.set(81.175);
        c.maxAchievableStrafeVelocity.set(66.8431);
        c.naturalForwardDeceleration.set(30.3333);
        c.naturalStrafeDeceleration.set(62.58098);
    });

    /** Plain Pedro follower, for the Pedro tuners (no Robot, no HardwareQueue). */
    public static Follower create(HardwareMap h) {
        return create(h, new Mecanum(h, mecanumConfig));
    }

    /** Robot follower: drive writes go through the HardwareQueue. */
    public static Follower create(HardwareMap h, HardwareQueue queue, Sensors sensors) {
        return create(h, new QueuedMecanum(h, mecanumConfig, queue, sensors));
    }

    private static Follower create(HardwareMap h, Mecanum drivetrain) {
        // Keep the pose carried over from auto in teleop; zero it everywhere else.
        pinpointConfig.resetMode.set(Globals.RUNMODE == RunMode.TELEOP
                ? PinpointLocalizer.ResetMode.NONE
                : PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        PinpointLocalizer localizer = new PinpointLocalizer(h, pinpointConfig);
        if (PINPOINT_FAST_I2C) setFastI2c(h);
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }

    /** The Pinpoint's one bulk read per loop is ~4x faster at 400 kHz than the hub's 100 kHz default. */
    public static boolean PINPOINT_FAST_I2C = true;

    private static void setFastI2c(HardwareMap h) {
        Object client = h.get(GoBildaPinpointDriver.class, pinpointConfig.name.get()).getDeviceClient();
        if (client instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) client).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        } else {
            RobotLog.ww("Constants", "Pinpoint I2C client is %s, bus speed left at default", client.getClass());
        }
    }
}
