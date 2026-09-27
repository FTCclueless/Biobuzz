package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Lerp;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
public class nMergeLocalizer extends Localizer {

    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;
    public static boolean constantCorrection = false;
    public static boolean usePinpoint = true;
    public static boolean useCamera = true;

    private Pose2d lastPinpointPose, pastPose;
    private long lastPinpointPollNanos;
    public static long pinpointPollGapMs = 1000;
    public static double pinpointResetDist = 5.0;

    // EKF
    private final RobotEKF ekf;
    public static double Q_X     = 0.1;
    public static double Q_Y     = 0.1;
    public static double Q_THETA = 0.1;

    private double py;
    private double px;
    private double pt;

    // Camera
    private Pose2d estimatedCameraPose = new Pose2d(0, 0, 0);
    public int numberOfTimesRelocalizedWithCamera = 0;
    public static double cameraFilterFactor = 0.2, cameraSmoothFactor = 0.02;
    public static int frameRequirement = 3;
    private int consecutiveFrames = 0;
    private int notVisibleCooldown = 0;
    public static int notVisibleCooldownInc = 10;
    public static double maxVisionErrorThresh = 10;

    public nMergeLocalizer(HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(3.391, 0.582, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        lastPinpointPollNanos = System.nanoTime();
        pinpoint.update();
        Pose2d p = new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
        TelemetryUtil.packet.put("Pinpoint start", String.format(Locale.US, "%.3f %.3f %.3f", p.x, p.y, p.heading));
        super.setPoseEstimate(p);
        lastPinpointPose = p;

        ekf = new RobotEKF(p, Q_X, Q_Y, Q_THETA);
        currentPose = p.clone();
    }

    public void update() {
        long currentTimeNanos = System.nanoTime();
        double loopTime = (double)(currentTimeNanos - lastTime) / 1.0E9;
        lastTime = currentTimeNanos;

        // 3 WHEEL ODOMETRY

        double deltaLeft  = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack  = encoders[2].getDelta();
        double leftY      = encoders[0].y;
        double rightY     = encoders[1].y;
        double backX      = encoders[2].x;

        double deltaHeading = (deltaRight - deltaLeft) / (leftY - rightY);
        relDeltaY = deltaBack - deltaHeading * backX;
        relDeltaX = (deltaRight * leftY - deltaLeft * rightY) / (leftY - rightY);
        distanceTraveled += Math.sqrt(relDeltaX * relDeltaX + relDeltaY * relDeltaY);

        Pose2d relDelta = new Pose2d(relDeltaX, relDeltaY, deltaHeading);

        // EKF PREDICT
        ekf.predict(relDeltaX,relDeltaY,deltaHeading);

        // EKF UPDATE — PINPOINT

        if (usePinpoint && (currentTimeNanos - lastPinpointPollNanos >= pinpointPollGapMs * 1000_000 || constantCorrection)) {
            pinpoint.update();
            px = pinpoint.getPosX();
            py = pinpoint.getPosY();
            pt = pinpoint.getHeading();
            Pose2d currpinpoint = new Pose2d(px,py,pt);
            if (pastPose != null && lastPinpointPose != null && currpinpoint != null) {
                Pose2d newpinpoint = MergeLocalizer.offsetPoseUsingGlobalDelta(pastPose, lastPinpointPose, currpinpoint);
                Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
                DashboardUtil.drawRobot(fieldOverlay, new Pose2d(newpinpoint.x, newpinpoint.y, newpinpoint.heading), this.expectedColor);
                if (consecutiveFrames == 0 && Math.hypot(px - ekf.getX(), py - ekf.getY()) > pinpointResetDist) {
                    Log.i("Localization Test", "ERROR IS TERRIBLE LOCK IN ASAP");
                    ekf.resetPose(new Pose2d(newpinpoint.x, newpinpoint.y, newpinpoint.heading));
                } else {
                    ekf.updatePinpoint(newpinpoint.x, newpinpoint.y, newpinpoint.heading);
                }
            }
            lastPinpointPose = currpinpoint.clone();
            pastPose = ekf.getPose();
            lastPinpointPollNanos = currentTimeNanos;
        }
        /*
        if (usePinpoint && (currentTimeNanos - lastPinpointPollNanos >= pinpointPollGapMs * 1000_000)) {
            pinpoint.update();
            px = pinpoint.getPosX();
            py = pinpoint.getPosY();
            pt = pinpoint.getHeading();
            if (Math.hypot(px - ekf.getX(), py - ekf.getY()) > pinpointResetDist && consecutiveFrames == 0 ) {
                ekf.resetPose(new Pose2d(px, py, pt));
            } else {
                ekf.updatePinpoint(px, py, pt);
            }
            Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
            DashboardUtil.drawRobot(fieldOverlay, new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading()), this.expectedColor);
        }
        */


        // CAMERA
        TelemetryUtil.packet.put("Vision is not null", drivetrain.vision != null);

        if (drivetrain.vision != null) {
            TelemetryUtil.packet.put("Vision Camera State", drivetrain.vision.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
            TelemetryUtil.packet.put("Vision Processor Enabled", drivetrain.vision.visionPortal.getProcessorEnabled(drivetrain.vision.aprilTagProcessor));
        }

        if (useCamera && Math.hypot(Globals.ROBOT_VELOCITY.x, Globals.ROBOT_VELOCITY.y) < 10 && drivetrain.vision != null && drivetrain.vision.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && drivetrain.vision.visionPortal.getProcessorEnabled(drivetrain.vision.aprilTagProcessor)) {
            Pose2d cameraResult = drivetrain.vision.update();
            if (cameraResult != null) {
                if (estimatedCameraPose == null) estimatedCameraPose = cameraResult;
                else estimatedCameraPose = new Pose2d(
                        Lerp.lerp(estimatedCameraPose.x, cameraResult.x, cameraFilterFactor),
                        Lerp.lerp(estimatedCameraPose.y, cameraResult.y, cameraFilterFactor),
                        Lerp.lerpAngle(estimatedCameraPose.heading, cameraResult.heading, cameraFilterFactor)
                );
                consecutiveFrames++;
                notVisibleCooldown = notVisibleCooldownInc;
            } else {
                --notVisibleCooldown;
                if (notVisibleCooldown < 0) {
                    estimatedCameraPose = null;
                    consecutiveFrames = 0;
                }
            }
            if (estimatedCameraPose != null) {
                long frameAcquisitionNanoTime = drivetrain.vision.frameAcquisitionNanoTime;
                if (nanoTimes.size() > 5
                        && consecutiveFrames >= frameRequirement
                        && Math.hypot(estimatedCameraPose.x - ekf.getX(), estimatedCameraPose.y - ekf.getY()) < maxVisionErrorThresh) {

                    findPastInterpolatedPose(frameAcquisitionNanoTime);

                    // Compute innovation against where the robot was in the past, not where it is now so we aren't influenced by process delay
                    double innovX     = estimatedCameraPose.x     - interpolatedPastPose.x;
                    double innovY     = estimatedCameraPose.y     - interpolatedPastPose.y;
                    double innovTheta = AngleUnit.normalizeRadians(
                            estimatedCameraPose.heading - interpolatedPastPose.heading);

                    // Apply the correction from past pose
                    ekf.updateSensorWithInnovation(innovX, innovY, innovTheta);

                    numberOfTimesRelocalizedWithCamera++;

                    TelemetryUtil.packet.put("Vision : estimatedCameraPose", estimatedCameraPose);
                    TelemetryUtil.packet.put("Vision : pastPose", interpolatedPastPose);
                    TelemetryUtil.packet.put("Vision : newPose", ekf.getPose());

                    Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
                    DashboardUtil.drawRobot(fieldOverlay, interpolatedPastPose, "#ff8000", 1);
                    DashboardUtil.drawRobot(fieldOverlay, ekf.getPose(), "#0000ff", 4);
                    DashboardUtil.drawRobot(fieldOverlay, estimatedCameraPose, "#000000", 2);
                }
            }
        }

        Pose2d ekfPose = ekf.getPose();
        ekfPose.x = Utils.minMaxClip(ekfPose.x, -72, 72);
        ekfPose.y = Utils.minMaxClip(ekfPose.y, -72, 72);

        currentPose = ekfPose;
        x       = currentPose.x;
        y       = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0, relDelta);
        nanoTimes.add(0, currentTimeNanos);
        poseHistory.add(0, currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
        ekf.resetPose(pose);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.x, pose.y, AngleUnit.RADIANS, pose.heading));
        lastPinpointPose = pastPose = pose.clone();
        estimatedCameraPose = null;
        consecutiveFrames   = 0;
    }

    public double getInstantaneousAngularVel() {
        return poseHistory.size() >= 2
                ? (poseHistory.get(0).heading - poseHistory.get(1).heading) / (nanoTimes.get(0) - nanoTimes.get(1))
                : 0;
    }

    public void relocalizeWithVision() {
        estimatedCameraPose = drivetrain.vision.update();
        if (estimatedCameraPose != null) {
            setPoseEstimate(estimatedCameraPose);
        }
    }

    public void updateField() {
        TelemetryUtil.packet.put(this.getClass().getSimpleName() + " x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName() + " y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName() + " heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName() + " distance", distanceTraveled);
        TelemetryUtil.packet.put(this.getClass().getSimpleName() + " velocity", relCurrentVel);

        TelemetryUtil.packet.put("Pinpoint x", px);
        TelemetryUtil.packet.put("Pinpoint y", py);
        TelemetryUtil.packet.put("Pinpoint heading", pt);

        TelemetryUtil.packet.put("EKF P[0][0] (x var)",     ekf.getCovarianceCopy()[0][0]);
        TelemetryUtil.packet.put("EKF P[1][1] (y var)",     ekf.getCovarianceCopy()[1][1]);
        TelemetryUtil.packet.put("EKF P[2][2] (theta var)", ekf.getCovarianceCopy()[2][2]);

        TelemetryUtil.packet.put("Vision x", estimatedCameraPose != null ? estimatedCameraPose.x : "na");
        TelemetryUtil.packet.put("Vision y", estimatedCameraPose != null ? estimatedCameraPose.y : "na");
        TelemetryUtil.packet.put("Vision heading", estimatedCameraPose != null ? estimatedCameraPose.heading : "na");
        TelemetryUtil.packet.put("Vision : useCamera", useCamera);
        TelemetryUtil.packet.put("Vision : consecutive frames", consecutiveFrames);
        TelemetryUtil.packet.put("Vision : relocalize count", numberOfTimesRelocalizedWithCamera);

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, this.color);
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
    }
}
