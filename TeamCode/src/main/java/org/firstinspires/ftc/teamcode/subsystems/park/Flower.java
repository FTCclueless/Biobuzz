package org.firstinspires.ftc.teamcode.subsystems.park;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

@Config
public class Flower {
    private final PriorityCRServo flower;

    private double power = 0;

    public Flower(Robot robot) {
        flower = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "park1"), robot.hardwareMap.get(CRServo.class, "park2")},
            "park", PriorityCRServo.ServoType.AXON_MAX,
            new boolean[]{false, true},
            2, 2
        );

        robot.hardwareQueue.addDevice(flower);
    }

    public void update() {
        flower.setTargetPower(power);
        TelemetryUtil.packet.put("Park : power", this.power);
    }

    public void setPower(double p) {
        power = p;
    }
}
