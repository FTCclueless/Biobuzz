package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Size;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Arducam (any UVC webcam) + FTC SDK VisionPortal + AprilTagProcessor, for the BIOBUZZ tags.
 *
 * Robot configuration: add a Webcam named CAMERA_NAME on the Control Hub's USB port.
 *
 * What you get per tag:
 *  - the SDK's camera-relative ftcPose (range / bearing / yaw...),
 *  - the tag's position in the ROBOT frame (camera mount applied),
 *  - the tag's position in the FIELD frame (robot pose applied).
 *
 * No field-pose (robotPose) localization: the BIOBUZZ tags are on the bottom of the HIVE cells,
 * and the hives tip, so a tag's field position isn't fixed. Fuse tags into odometry only once
 * a hive tracker knows the hive state (biobuzz-code-plan.md, Phase 2).
 *
 * Tag map (§9.9): 30-33 red far-side cell, 34-37 red audience-side, 38-41 blue audience-side,
 * 42-45 blue far-side. 36h11, 3.25 in.
 */
@Config
public class ArducamAprilTags {
    public static String CAMERA_NAME = "camera";
    public static int WIDTH = 640, HEIGHT = 480;
    /** MJPEG gets full frame rate over USB 2 on most Arducams; YUY2 often caps at 5-10 fps at 640x480. */
    public static boolean MJPEG = true;
    public static boolean STREAM_TO_DASHBOARD = true;

    public static double TAG_SIZE_IN = 3.25;

    /**
     * Lens intrinsics for WIDTH x HEIGHT, in pixels. Leave FX <= 0 to use the SDK's built-in
     * calibration for your camera (the SDK logs a warning if it has none, and poses will be off).
     * Calibrate with 3DF Zephyr or calib.io and paste the numbers here for accurate range.
     */
    public static double FX = -1, FY = -1, CX = -1, CY = -1;

    /** Manual exposure. Short = less motion blur = detections while driving. Read when the stream starts. */
    public static boolean MANUAL_EXPOSURE = true;
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 200;

    /** 1 = full res (far tags), 2-3 = faster, shorter range. Live-tunable. */
    public static double DECIMATION = 2;
    public static double MIN_DECISION_MARGIN = 30;

    // Camera mount relative to robot center (in / deg). The tags face DOWN, so the camera tilts UP.
    public static double CAM_FORWARD = 0, CAM_LEFT = 0, CAM_HEIGHT = 10;
    public static double CAM_PITCH_UP = 30;
    public static double CAM_YAW = 0;

    /** One detection, in all three frames. */
    public static class Tag {
        public final int id;
        public final String group;
        public final AprilTagPoseFtc ftcPose;  // camera frame, straight from the SDK
        public final double robotX, robotY, robotZ;   // robot frame: +x fwd, +y left, +z up
        public final Vector2 field;            // field frame (x, y)
        public final double decisionMargin;
        public final long frameNanos;

        Tag(int id, String group, AprilTagPoseFtc p, double rx, double ry, double rz, Vector2 field,
            double margin, long frameNanos) {
            this.id = id;
            this.group = group;
            this.ftcPose = p;
            this.robotX = rx;
            this.robotY = ry;
            this.robotZ = rz;
            this.field = field;
            this.decisionMargin = margin;
            this.frameNanos = frameNanos;
        }
    }

    public final AprilTagProcessor processor;
    public final VisionPortal portal;
    private boolean controlsSet = false;
    private double lastDecimation = -1;
    private final List<Tag> tags = new ArrayList<>();

    public static String groupOf(int id) {
        if (id >= 30 && id <= 33) return "RED_FAR";
        if (id >= 34 && id <= 37) return "RED_AUDIENCE";
        if (id >= 38 && id <= 41) return "BLUE_AUDIENCE";
        if (id >= 42 && id <= 45) return "BLUE_FAR";
        return "UNKNOWN";
    }

    public static AprilTagLibrary biobuzzLibrary() {
        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) {
            b.addTag(id, groupOf(id) + "_" + id, TAG_SIZE_IN, DistanceUnit.INCH);
        }
        return b.build();
    }

    public ArducamAprilTags(HardwareMap hardwareMap) {
        AprilTagProcessor.Builder pb = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(biobuzzLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true);
        if (FX > 0) pb.setLensIntrinsics(FX, FY, CX, CY);
        processor = pb.build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(WIDTH, HEIGHT))
                .setStreamFormat(MJPEG ? VisionPortal.StreamFormat.MJPEG : VisionPortal.StreamFormat.YUY2)
                .addProcessor(processor)
                .build();

        if (STREAM_TO_DASHBOARD && TelemetryUtil.dashboard != null) {
            TelemetryUtil.dashboard.startCameraStream(portal, 15);
        }
    }

    /**
     * Camera controls only exist once the camera is STREAMING. Setting them in the constructor
     * (like the old Vision class did) usually throws and silently leaves auto-exposure on.
     */
    private void trySetControls() {
        if (controlsSet || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        try {
            if (MANUAL_EXPOSURE) {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
                exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                GainControl gain = portal.getCameraControl(GainControl.class);
                gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN)));
            }
        } catch (Exception e) {
            RobotLog.ww("ArducamAprilTags", "Could not set exposure/gain: %s", e);
        }
        controlsSet = true;
    }

    /** Call once per loop. Returns this frame's tags (empty if no new frame). */
    public List<Tag> update() {
        trySetControls();
        if (DECIMATION != lastDecimation) {
            processor.setDecimation((float) DECIMATION);
            lastDecimation = DECIMATION;
        }

        tags.clear();
        List<AprilTagDetection> fresh = processor.getFreshDetections();
        if (fresh != null) {
            for (AprilTagDetection d : fresh) {
                if (d.metadata == null || d.ftcPose == null) continue; // not a BIOBUZZ tag
                if (d.decisionMargin < MIN_DECISION_MARGIN) continue;
                tags.add(toTag(d));
            }
        }
        telemetry();
        return tags;
    }

    private Tag toTag(AprilTagDetection d) {
        AprilTagPoseFtc p = d.ftcPose;
        // ftcPose camera frame: +x right, +y out of the lens, +z up (in the image).
        // Tilt by pitch (up), then yaw, then add the mount offset -> robot frame.
        double pitch = Math.toRadians(CAM_PITCH_UP);
        double fwd = p.y * Math.cos(pitch) - p.z * Math.sin(pitch);
        double up = p.y * Math.sin(pitch) + p.z * Math.cos(pitch);
        double left = -p.x;
        double yaw = Math.toRadians(CAM_YAW);
        double rx = CAM_FORWARD + fwd * Math.cos(yaw) - left * Math.sin(yaw);
        double ry = CAM_LEFT + fwd * Math.sin(yaw) + left * Math.cos(yaw);
        double rz = CAM_HEIGHT + up;
        Vector2 field = GroundProjection.robotToField(new Vector2(rx, ry),
                ROBOT_POSITION.x, ROBOT_POSITION.y, ROBOT_POSITION.heading);
        return new Tag(d.id, groupOf(d.id), p, rx, ry, rz, field, d.decisionMargin, d.frameAcquisitionNanoTime);
    }

    private void telemetry() {
        TelemetryUtil.packet.put("AprilTag : camera state", portal.getCameraState().toString());
        TelemetryUtil.packet.put("AprilTag : fps", portal.getFps());
        TelemetryUtil.packet.put("AprilTag : tags", tags.size());
        Canvas c = TelemetryUtil.packet.fieldOverlay();
        c.setStroke("#AA00FF");
        c.setStrokeWidth(1);
        for (Tag t : tags) {
            TelemetryUtil.packet.put("AprilTag " + t.id + " : range (in)", t.ftcPose.range);
            TelemetryUtil.packet.put("AprilTag " + t.id + " : bearing (deg)", Math.toDegrees(t.ftcPose.bearing));
            TelemetryUtil.packet.put("AprilTag " + t.id + " : height (in)", t.robotZ);
            c.strokeRect(t.field.x - 1.6, t.field.y - 1.6, 3.25, 3.25);
        }
    }

    public List<Tag> getTags() { return tags; }

    /** Average field position of the tags seen this frame in one hive-cell group, or null. */
    public Vector2 groupCenter(String group) {
        double sx = 0, sy = 0;
        int n = 0;
        for (Tag t : tags) {
            if (t.group.equals(group)) { sx += t.field.x; sy += t.field.y; n++; }
        }
        return n == 0 ? null : new Vector2(sx / n, sy / n);
    }

    public void close() { portal.close(); }
}
