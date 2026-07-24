package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.List;

@Config
public class Sensors {
    private final Robot robot;
    private final List<LynxModule> allHubs;

    public double loopTime;
    private long currentTime;

    private double voltage;
    public long voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = 0;
    private final VoltageSensor voltageSensor;

    public Sensors(Robot robot) {
        this.robot = robot;

        currentTime = System.nanoTime();
        voltageSensor = robot.hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();


        allHubs = robot.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        long lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        if (currentTime - lastVoltageUpdatedTime > voltageUpdateTime * 1e6) {
            voltage = voltageSensor.getVoltage();
            lastVoltageUpdatedTime = currentTime;
        }

        TelemetryUtil.packet.put("Sensors: Voltage", voltage);
    }


    public double getVoltage() {
        return voltage;
    }
}

