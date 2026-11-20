package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.ArducamAprilTags;

import java.util.Locale;

/**
 * Arducam AprilTag check. Runs only the camera (no motors), so it's safe on the bench.
 *
 * Hold a BIOBUZZ tag (36h11, ID 30-45) in front of the camera. Driver Station shows range /
 * bearing / yaw per tag; Dashboard shows the camera stream (Camera panel) and the same numbers.
 * Check range against a tape measure: if it's off by a constant factor, the lens intrinsics
 * (ArducamAprilTags.FX/FY/CX/CY) or TAG_SIZE_IN are wrong.
 */
@TeleOp(name = "Arducam AprilTag Test", group = "test")
public class ArducamAprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryUtil.setup();
        ArducamAprilTags cam = new ArducamAprilTags(hardwareMap);

        while (opModeInInit()) {
            cam.update();
            telemetry.addData("camera", cam.portal.getCameraState());
            telemetry.addLine("Press start (tags are shown in init too)");
            showTags(cam);
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }

        while (opModeIsActive()) {
            cam.update();
            telemetry.addData("camera", cam.portal.getCameraState());
            telemetry.addData("fps", "%.1f", cam.portal.getFps());
            showTags(cam);
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }
        cam.close();
    }

    private void showTags(ArducamAprilTags cam) {
        telemetry.addData("tags", cam.getTags().size());
        for (ArducamAprilTags.Tag t : cam.getTags()) {
            telemetry.addLine(String.format(Locale.US,
                    "ID %d %s | range %.1f in  bearing %.1f°  yaw %.1f° | robot (%.1f, %.1f, %.1f) | margin %.0f",
                    t.id, t.group, t.ftcPose.range, Math.toDegrees(t.ftcPose.bearing), Math.toDegrees(t.ftcPose.yaw),
                    t.robotX, t.robotY, t.robotZ, t.decisionMargin));
        }
    }
}
