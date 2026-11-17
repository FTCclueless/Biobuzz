package org.firstinspires.ftc.teamcode.pedroPathing.control;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;

public class Tuning {
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(hardwareMap -> {
            Constants.pinpointConfig.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
            return new PinpointLocalizer(hardwareMap, Constants.pinpointConfig);
        }, hardwareMap -> new Mecanum(hardwareMap, Constants.mecanumConfig));
    }
}
