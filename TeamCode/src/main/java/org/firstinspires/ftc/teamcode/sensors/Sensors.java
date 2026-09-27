package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_ACCELERATION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.List;

/** Drive sensors, bulk caching, battery voltage, and odometry. */
@Config
public class Sensors {
    private final Robot robot;
    private final List<LynxModule> allHubs;
    private final VoltageSensor voltageSensor;
    private final int[] odoWheelPositions = {0, 0, 0};

    public double loopTime;
    private long currentTime;
    private double voltage;
    public static long voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = 0;

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

        odoWheelPositions[0] = robot.drivetrain.leftFront.motor[0].getCurrentPosition();
        odoWheelPositions[1] = robot.drivetrain.rightFront.motor[0].getCurrentPosition();
        odoWheelPositions[2] = robot.drivetrain.leftRear.motor[0].getCurrentPosition();

        robot.drivetrain.localizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.localizer.update();
        robot.drivetrain.nMergeLocalizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.nMergeLocalizer.update();

        ROBOT_POSITION = robot.drivetrain.nMergeLocalizer.getPoseEstimate();
        ROBOT_VELOCITY = robot.drivetrain.nMergeLocalizer.getRelativePoseVelocity();
        ROBOT_GLOBAL_VELOCITY = robot.drivetrain.nMergeLocalizer.getGlobalVelocity();
        ROBOT_GLOBAL_ACCELERATION = robot.drivetrain.nMergeLocalizer.getGlobalAccel();

        if (currentTime - lastVoltageUpdatedTime > voltageUpdateTime * 1e6) {
            voltage = voltageSensor.getVoltage();
            lastVoltageUpdatedTime = currentTime;
        }

        TelemetryUtil.packet.put("Sensors: Voltage", voltage);
        DashboardUtil.drawRobot(TelemetryUtil.packet.fieldOverlay(), ROBOT_POSITION, "#00ff00");
        LogUtil.driveCurrentX.set(ROBOT_POSITION.x);
        LogUtil.driveCurrentY.set(ROBOT_POSITION.y);
        LogUtil.driveCurrentAngle.set(ROBOT_POSITION.heading);
    }

    public double getVoltage() {
        return voltage;
    }
}
