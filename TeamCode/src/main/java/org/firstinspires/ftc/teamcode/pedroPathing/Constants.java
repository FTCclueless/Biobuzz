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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
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
        c.xPodOffset.set(3.391);
        c.yPodOffset.set(0.582);
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

    public static Follower create(HardwareMap h) {
        // Keep the pose carried over from auto in teleop; zero it everywhere else.
        pinpointConfig.resetMode.set(Globals.RUNMODE == RunMode.TELEOP
                ? PinpointLocalizer.ResetMode.NONE
                : PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        return new Follower(new PinpointLocalizer(h, pinpointConfig), new Mecanum(h, mecanumConfig),
                new Foresight(foresightConfig));
    }
}
