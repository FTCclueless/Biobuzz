package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
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
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.xPodOffset.set(5.793267717);
        c.yPodOffset.set(0.629921);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2488132775871428);
                Controller secondaryTranslationalForward = Controller.proportional(0.09192985765522656);
                Controller primaryTranslationalLateral = Controller.proportional(0.3933395931844317);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14532846945414796);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012218102822423109));
                c.brake.set(Controller.proportionalFeedforward(0.010385387399059642));

                c.headingFeedback.set(Controller.proportional(4.160062788277549));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0429748151459226, 0.006620055070319795));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07341984794255606, 0.0312803504972456));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015587374537178818, 0.0023187603403183707));

                c.maxAchievableForwardVelocity.set(81.48094428603838);
                c.maxAchievableStrafeVelocity.set(68.58553802709417);
                c.naturalForwardDeceleration.set(36.27108691045682);
                c.naturalStrafeDeceleration.set(60.97113817838488);
            }
    );

    /** Plain Pedro follower, for the Pedro tuners (no Robot, no HardwareQueue). */
    public static Follower create(HardwareMap h) {
        // Tuners never set Globals.RUNMODE, so it holds the last OpMode's value: always reset here.
        return create(h, new Mecanum(h, mecanumConfig), PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    }

    /** Robot follower: drive writes go through the HardwareQueue. */
    public static Follower create(HardwareMap h, HardwareQueue queue, Sensors sensors) {
        // Keep the pose carried over from auto in teleop; zero it everywhere else.
        return create(h, new QueuedMecanum(h, mecanumConfig, queue, sensors),
                Globals.RUNMODE == RunMode.TELEOP
                        ? PinpointLocalizer.ResetMode.NONE
                        : PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    }

    private static Follower create(HardwareMap h, Mecanum drivetrain, PinpointLocalizer.ResetMode reset) {
        pinpointConfig.resetMode.set(reset);
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
