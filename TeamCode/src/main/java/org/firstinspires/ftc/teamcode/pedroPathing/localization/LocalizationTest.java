package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp(group = "1")
public class LocalizationTest extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void init_loop() {
        follower.update();
        updatePoseTelemetry();
        telemetry.update();
    }

    private void updatePoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pinpoint X (in)", pose.x());
        telemetry.addData("Pinpoint Y (in)", pose.y());
        telemetry.addData("Pinpoint Heading (deg)", Math.toDegrees(pose.heading()));
        // Keep the field drawing and dashboard values in the same packet.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Pinpoint X (in)", pose.x());
        packet.put("Pinpoint Y (in)", pose.y());
        packet.put("Pinpoint Heading (deg)", Math.toDegrees(pose.heading()));
        packet.put("Loop Time Hz", 1000 / loopTime);
        packet.put("Mode", follower.mode());
        packet.put("Manual?", follower.manual());
        DashboardUtil.drawRobot(packet.fieldOverlay(),
                new Pose2d(pose.x(), pose.y(), pose.heading()), "#00C853");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x); // +strafe = left, +turn = CCW
        follower.update();
        telemetry.addData("Loop Time Hz", 1000/loopTime);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Manual?", follower.manual());
        telemetry.addData("Pose", follower.pose());
        updatePoseTelemetry();
        telemetry.update();
    }
}
