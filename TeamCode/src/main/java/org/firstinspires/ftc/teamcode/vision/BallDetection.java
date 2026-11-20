package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Limelight 3A neural detector -> field positions of POLLEN / NECTAR -> best cluster to drive to.
 *
 * Limelight setup (web UI at http://172.29.0.1:5801):
 *  - Pipeline type: Neural Detector, in slot PIPELINE. Upload the .tflite + labels.
 *  - Labels must contain "pollen" / "yellow", "red", or "blue" (case-insensitive). See classify().
 *  - Low exposure (motion blur kills detections while driving), confidence ~0.3.
 *
 * Every value here is live-tunable from Dashboard -> BallDetection.
 */
@Config
public class BallDetection {
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;

    // Camera mount. MEASURE these; the distance estimate is only as good as height + pitch.
    public static double CAM_FORWARD = 8.9;      // in, lens forward of robot center
    public static double CAM_LEFT = 0.0;         // in, lens left of robot center
    public static double CAM_HEIGHT = 10.0;      // in, lens above the floor
    public static double CAM_PITCH_DOWN = 20.0;  // deg, tilt toward the floor
    public static double CAM_YAW = 0.0;          // deg, CCW from robot forward (180 if facing backwards)

    public static double POLLEN_RADIUS = 1.4;    // §9.8: 2.8 in diameter
    public static double NECTAR_RADIUS = 1.8;    // §9.8: 3.6 in diameter
    public static double MAX_RANGE = 90.0;       // in. Beyond this the ty->distance math is noise

    public static double MIN_CONFIDENCE = 0.3;
    public static double MAX_STALENESS_MS = 100;
    public static boolean LATENCY_COMPENSATION = true;

    // Tracker
    public static double MERGE_DIST = 3.0;
    public static int MIN_HITS = 2;
    public static long TRACK_TIMEOUT_MS = 1500;

    // Clustering (see BallClusterer for what each does)
    public static double LINK_DIST = 7.0;
    public static double NECTAR_WEIGHT = 1.0;
    public static boolean COLLECT_NECTAR = true;
    public static double DISTANCE_PENALTY = 0.01;
    public static double SWITCH_MARGIN = 0.75;
    public static double APPROACH_STANDOFF = 14.0;

    public static boolean DRAW = true;

    private final Limelight3A limelight;
    private final BallTracker tracker = new BallTracker();
    private final BallClusterer clusterer = new BallClusterer();
    private final List<BallClusterer.Ball> frame = new ArrayList<>();
    private int lastPipeline = -1;
    private long lastResultTimestamp = -1;
    private BallClusterer.Target target = null;

    public BallDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        setPipeline();
    }

    public void start() { limelight.start(); }

    public void stop() { limelight.stop(); }

    private void setPipeline() {
        if (PIPELINE != lastPipeline) {
            limelight.pipelineSwitch(PIPELINE);
            lastPipeline = PIPELINE;
        }
    }

    private void applyParams() {
        tracker.mergeDist = MERGE_DIST;
        tracker.minHits = MIN_HITS;
        tracker.timeoutMs = TRACK_TIMEOUT_MS;
        clusterer.linkDist = LINK_DIST;
        clusterer.nectarWeight = NECTAR_WEIGHT;
        clusterer.collectNectar = COLLECT_NECTAR;
        clusterer.distancePenalty = DISTANCE_PENALTY;
        clusterer.switchMargin = SWITCH_MARGIN;
        clusterer.approachStandoff = APPROACH_STANDOFF;
        clusterer.isRed = Globals.isRed;
    }

    /** Map a model label to a ball type. Returns null for labels we don't use. */
    public static BallClusterer.Type classify(String label) {
        if (label == null) return null;
        String l = label.toLowerCase(Locale.US);
        if (l.contains("pollen") || l.contains("yellow")) return BallClusterer.Type.POLLEN;
        if (l.contains("red")) return BallClusterer.Type.RED_NECTAR;
        if (l.contains("blue")) return BallClusterer.Type.BLUE_NECTAR;
        return null;
    }

    /** Call once per loop, after the drivetrain has updated ROBOT_POSITION. */
    public void update() {
        applyParams();
        setPipeline();
        long now = System.currentTimeMillis();
        frame.clear();

        LLResult result = limelight.getLatestResult();
        boolean fresh = result != null && result.isValid()
                && result.getStaleness() < MAX_STALENESS_MS
                && result.getControlHubTimeStamp() != lastResultTimestamp;

        if (fresh) {
            lastResultTimestamp = result.getControlHubTimeStamp();

            // Where the robot was when the frame was captured, not where it is now.
            Pose2d pose = ROBOT_POSITION;
            if (LATENCY_COMPENSATION) {
                double latency = (result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness()) / 1000.0;
                Pose2d v = ROBOT_GLOBAL_VELOCITY;
                pose = new Pose2d(pose.x - v.x * latency, pose.y - v.y * latency, pose.heading - v.heading * latency);
            }

            for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
                if (d.getConfidence() < MIN_CONFIDENCE) continue;
                BallClusterer.Type type = classify(d.getClassName());
                if (type == null) continue;
                double tx = d.getTargetXDegrees(), ty = d.getTargetYDegrees();
                if (Double.isNaN(tx) || Double.isNaN(ty)) continue;

                double radius = type == BallClusterer.Type.POLLEN ? POLLEN_RADIUS : NECTAR_RADIUS;
                Vector2 rel = GroundProjection.toRobotFrame(tx, ty, CAM_FORWARD, CAM_LEFT, CAM_HEIGHT,
                        CAM_PITCH_DOWN, CAM_YAW, radius, MAX_RANGE);
                if (rel == null) continue;
                frame.add(new BallClusterer.Ball(
                        GroundProjection.robotToField(rel, pose.x, pose.y, pose.heading), type));
            }
            tracker.update(frame, now);
        } else {
            tracker.prune(now);
        }

        target = clusterer.target(tracker.confirmed(), ROBOT_POSITION.toVec2());
        telemetry(fresh);
    }

    private void telemetry(boolean fresh) {
        TelemetryUtil.packet.put("Balls : limelight connected", limelight.isConnected());
        TelemetryUtil.packet.put("Balls : fresh frame", fresh);
        TelemetryUtil.packet.put("Balls : this frame", frame.size());
        TelemetryUtil.packet.put("Balls : tracked", tracker.confirmed().size());
        if (target != null) {
            TelemetryUtil.packet.put("Balls : target size", target.cluster.balls.size());
            TelemetryUtil.packet.put("Balls : target x", target.cluster.centroid.x);
            TelemetryUtil.packet.put("Balls : target y", target.cluster.centroid.y);
        } else {
            TelemetryUtil.packet.put("Balls : target size", 0);
        }
        if (!DRAW) return;

        Canvas c = TelemetryUtil.packet.fieldOverlay();
        c.setStrokeWidth(1);
        for (BallTracker.Track t : tracker.allTracks()) {
            String color = t.type == BallClusterer.Type.POLLEN ? "#FFC400"
                    : t.type == BallClusterer.Type.RED_NECTAR ? "#E53935" : "#1E88E5";
            c.setStroke(color);
            double r = t.type == BallClusterer.Type.POLLEN ? POLLEN_RADIUS : NECTAR_RADIUS;
            if (t.hits >= MIN_HITS) { c.setFill(color); c.fillCircle(t.pos.x, t.pos.y, r); }
            else c.strokeCircle(t.pos.x, t.pos.y, r);
        }
        if (target != null) {
            c.setStroke("#00E5FF");
            c.setStrokeWidth(2);
            c.strokeCircle(target.cluster.centroid.x, target.cluster.centroid.y, target.cluster.radius + 3);
            c.strokeLine(target.approach.x, target.approach.y, target.cluster.centroid.x, target.cluster.centroid.y);
        }
    }

    /** Best cluster + approach pose, or null if no allowed balls are tracked. */
    public BallClusterer.Target getTarget() { return target; }

    /** Approach pose of the best cluster, or null. */
    public Pose2d getTargetPose() {
        return target == null ? null : new Pose2d(target.approach, target.heading);
    }

    public List<BallClusterer.Ball> getTrackedBalls() { return tracker.confirmed(); }

    /** After driving through a cluster, forget it so we pick the next one. */
    public void markCollected(Vector2 center, double radius) {
        tracker.clearAround(center, radius);
        clusterer.reset();
        target = null;
    }

    public Limelight3A getLimelight() { return limelight; }
}
