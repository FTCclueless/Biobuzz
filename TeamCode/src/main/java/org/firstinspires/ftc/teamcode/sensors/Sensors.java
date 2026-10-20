package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_ACCELERATION;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.List;

@Config
public class Sensors {
    private final Robot robot;
    private final List<LynxModule> allHubs;

    public double loopTime;
    private long currentTime;

    //private double intakeCurrent;

    private double voltage;
    public static long voltageUpdateTime = 5000;
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

        robot.drivetrain.localizer.update(loopTime);
        ROBOT_POSITION = robot.drivetrain.localizer.getPose();
        ROBOT_VELOCITY = robot.drivetrain.localizer.getRelativeVelocity();
        ROBOT_GLOBAL_VELOCITY = robot.drivetrain.localizer.getGlobalVelocity();
        ROBOT_GLOBAL_ACCELERATION = robot.drivetrain.localizer.getGlobalAccel();

        //intakeCurrent = robot.intake.roller.getCurrent();

        if (currentTime - lastVoltageUpdatedTime > voltageUpdateTime * 1e6) {
            voltage = voltageSensor.getVoltage();
            lastVoltageUpdatedTime = currentTime;
        }

        TelemetryUtil.packet.put("Sensors: Voltage", voltage);
        TelemetryUtil.packet.put("Shooter : Hood launch angle (deg)", Math.toDegrees(robot.shooter.hood.getCurrentAngle() / Shooter.hoodGearRatio + Shooter.hoodSweep));
        //TelemetryUtil.packet.put("Intake : current (AMPS)", intakeCurrent);

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, ROBOT_POSITION, "#00ff00",
                robot.shooter.turret.getAngle(), "#00e000c0",
                robot.shooter.turret.getTargetAngle(), "#8000ff");

        LogUtil.driveCurrentX.set(ROBOT_POSITION.x);
        LogUtil.driveCurrentY.set(ROBOT_POSITION.y);
        LogUtil.driveCurrentAngle.set(ROBOT_POSITION.heading);
    }

    public double getVoltage() {
        return voltage;
    }
}
