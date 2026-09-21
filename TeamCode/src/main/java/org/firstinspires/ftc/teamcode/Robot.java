package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.photon.PhotonCore;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class Robot {
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;

    public Sensors sensors;
    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public Park park;

    private BooleanSupplier stopChecker = null;
    public ArrayList<Consumer<Canvas>> canvasDrawTasks = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) { this(hardwareMap, false); }

    public Robot(HardwareMap hardwareMap, boolean useVision) {
        PhotonCore.enable();

        this.hardwareMap = hardwareMap;
        hardwareQueue = new HardwareQueue();

        TelemetryUtil.setup();
        LogUtil.reset();

        sensors = new Sensors(this);
        drivetrain = new Drivetrain(this, useVision ? new Vision(hardwareMap) : null);
        intake = new Intake(this);
        shooter = new Shooter(this);
        park = new Park(this);
        sensors.resetTurretAngleEncoder();
    }

    public void update() {
        START_LOOP();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) return;

        sensors.update();

        drivetrain.update();
        intake.update();
        shooter.update();
        park.update();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) return;

        hardwareQueue.update();

        this.updateTelemetry();
    }

    public void setStopChecker(BooleanSupplier func) { this.stopChecker = func; }

    public void waitWhile(BooleanSupplier func) {
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && func.getAsBoolean());
    }

    public void waitWhileWithTimeout(BooleanSupplier func, long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && System.currentTimeMillis() - start < duration && func.getAsBoolean());
    }

    public void waitFor(long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && System.currentTimeMillis() - start < duration);
    }

    public void updateTelemetry() {
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        for (Consumer<Canvas> task : canvasDrawTasks) task.accept(canvas);

        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.packet.put("Photon Enabled", PhotonCore.isEnabled().get());

        TelemetryUtil.sendTelemetry();
        LogUtil.send();
    }
}
