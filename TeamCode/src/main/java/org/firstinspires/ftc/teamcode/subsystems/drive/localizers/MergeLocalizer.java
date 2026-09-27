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
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
public class MergeLocalizer extends Localizer {
    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(3.391, 0.582, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        lastPinpointPollNanos = System.nanoTime();
        pinpoint.update();
        Pose2d p = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
        TelemetryUtil.packet.put("Pinpoint start", String.format(Locale.US, "%.3f %.3f %.3f", p.x, p.y, p.heading));
        super.setPoseEstimate(p);
        lastPinpointMergePose = currentPose.clone();
        lastPinpointPose = p;
    }

    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d lastPinpointPose, lastPinpointMergePose;
    private long lastPinpointPollNanos;
    public static boolean constantCorrection = false;
    public static boolean usePinpoint = true;
    public static double pinpointPollDist = 12;
    public static long pinpointPollGapMs = 1000;

    // Camera
    private Pose2d estimatedCameraPose = new Pose2d(0,0,0);
    public static boolean useCamera = false;
    public int numberOfTimesRelocalizedWithCamera = 0;
    public static double cameraFilterFactor = 0.2, cameraSmoothFactor = 0.02;
    //how many frames the camera has to see consecutively before it updates the pose
    public static int frameRequirement = 3;
    private int consecutiveFrames = 0;
    private int notVisibleCooldown = 0;
    public static int notVisibleCooldownInc = 10;
    public static int maxVisionErrorThresh = 10;
    public static int maxVisionErrorThreshHeading = 20; //degrees
    public static int cameraSearch = 25; // number of loops

    public void update() {
        long currentTimeNanos = System.nanoTime();
        double loopTime = (double)(currentTimeNanos - lastTime)/1.0E9;
        lastTime = currentTimeNanos;

        // 3 WHEEL ODOMETRY

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        Pose2d relDelta = new Pose2d (relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime, relDelta, currentPose);

        // PINPOINT

        if ((usePinpoint && (currentTimeNanos - lastPinpointPollNanos >= pinpointPollGapMs * 1000_000 || currentPose.getDistanceFromPoint(lastPinpointMergePose) >= pinpointPollDist)) || constantCorrection) {
            Log.i("Localization Test", "pinpoint in use");
            lastPinpointPollNanos = currentTimeNanos;
            pinpoint.update();

            Pose2d globalPinpointEstimate = offsetPoseUsingGlobalDelta(lastPinpointMergePose, lastPinpointPose, new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading()));
            clipPoseToField(globalPinpointEstimate);
            lastPinpointPose = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
            currentPose = globalPinpointEstimate.clone();
            lastPinpointMergePose = globalPinpointEstimate.clone();
        } else {
            clipPoseToField(currentPose);
        }

        /*
        if (useCamera) {
            if (drivetrain.vision.visionPortal != null) {
                drivetrain.vision.visionPortal.setProcessorEnabled(drivetrain.vision.aprilTagProcessor, true);
            }

            estimatedCameraPose = drivetrain.vision.update();
            if (estimatedCameraPose != null) {
                setPoseEstimate(estimatedCameraPose);
                useCamera = false;
                Log.i("Vision", "Updated");
            }
        } else {
            if (drivetrain.vision.visionPortal != null) {
                drivetrain.vision.visionPortal.setProcessorEnabled(drivetrain.vision.aprilTagProcessor, false);
            }
        }

         */

        // Camera
        /*
        if (useCamera && drivetrain.vision != null) {
            drivetrain.vision.visionPortal.setProcessorEnabled(drivetrain.vision.aprilTagProcessor, currentPose.heading % (Math.PI * 2) > Math.PI / 2 && currentPose.heading % (Math.PI * 2) < Math.PI * 3 / 2);
            Log.i("Vision", "Heading is good stuff " + (currentPose.heading % (Math.PI * 2) > Math.PI / 2 && currentPose.heading % (Math.PI * 2) < Math.PI * 3 / 2));
        }
         */

        TelemetryUtil.packet.put("Vision is not null", drivetrain.vision != null);

        if (drivetrain.vision != null) {
            TelemetryUtil.packet.put("Vision Camera State", drivetrain.vision.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
            TelemetryUtil.packet.put("Vision Processor Enabled", drivetrain.vision.visionPortal.getProcessorEnabled(drivetrain.vision.aprilTagProcessor));
        }

        if (useCamera && Math.hypot(Globals.ROBOT_VELOCITY.x, Globals.ROBOT_VELOCITY.y) < 20 && drivetrain.vision != null && drivetrain.vision.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && drivetrain.vision.visionPortal.getProcessorEnabled(drivetrain.vision.aprilTagProcessor)) {
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
                Log.i("Vision", "After updating pose " + estimatedCameraPose);

                //if(consecutiveFrames >= frameRequirement) {
                //  consecutiveFrames = 0;
                //we want to find the last pinpoint/odo pose at the time that the camera was taken

                if (nanoTimes.size() > 5 && consecutiveFrames >= frameRequirement /*&& Math.abs(estimatedCameraPose.getErrorInX(currentPose)) < maxVisionErrorThresh && Math.abs(estimatedCameraPose.getErrorInY(currentPose)) < maxVisionErrorThresh && Math.abs(Utils.headingClip(estimatedCameraPose.heading - currentPose.heading)) < Math.toRadians(maxVisionErrorThreshHeading) */) {
                    findPastInterpolatedPose(frameAcquisitionNanoTime);
                    //then find the offset between that and the camera pose

                    //cameraOffsets = new Pose2d(pastPose.x - estimatedCameraPose.x, pastPose.y - estimatedCameraPose.y, pastPose.heading - estimatedCameraPose.heading);
                    Pose2d smoothCameraPose = new Pose2d(
                        Lerp.lerp(interpolatedPastPose.x, estimatedCameraPose.x, cameraSmoothFactor),
                        Lerp.lerp(interpolatedPastPose.y, estimatedCameraPose.y, cameraSmoothFactor),
                        Lerp.lerpAngle(interpolatedPastPose.heading, estimatedCameraPose.heading, cameraSmoothFactor)
                    );
                    Pose2d newPose = offsetPoseUsingGlobalDelta(currentPose, interpolatedPastPose, smoothCameraPose);

                    if (Math.hypot(newPose.x - currentPose.x, newPose.y - currentPose.y) > 10) LogUtil.drivePositionReset = true;
                    currentPose = newPose;
                    lastPinpointMergePose = offsetPoseUsingGlobalDelta(lastPinpointMergePose, interpolatedPastPose, smoothCameraPose);
                    poseHistory.replaceAll(now -> offsetPoseUsingGlobalDelta(now, interpolatedPastPose, smoothCameraPose));

                    numberOfTimesRelocalizedWithCamera++;


                    TelemetryUtil.packet.put("Vision : estimatedCameraPose", estimatedCameraPose);
                    TelemetryUtil.packet.put("Vision : pastPose", interpolatedPastPose);
                    TelemetryUtil.packet.put("Vision : newPose", newPose);

                    Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
                    DashboardUtil.drawRobot(fieldOverlay, interpolatedPastPose, "#ff8000", 1);
                    DashboardUtil.drawRobot(fieldOverlay, newPose, "#0000ff", 4);
                    DashboardUtil.drawRobot(fieldOverlay, estimatedCameraPose, "#000000", 2);
                }
            }
        }

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTimeNanos);
        poseHistory.add(0, currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public static Pose2d offsetPoseUsingGlobalDelta(Pose2d now, Pose2d offPrev, Pose2d offCurr) {
        Pose2d globalDelta = new Pose2d (
            offCurr.x - offPrev.x,
            offCurr.y - offPrev.y,
            offCurr.heading - offPrev.heading
        );

        Pose2d relDelta = new Pose2d (
            Math.cos(offPrev.heading) * globalDelta.x + Math.sin(offPrev.heading) * globalDelta.y,
            -Math.sin(offPrev.heading) * globalDelta.x + Math.cos(offPrev.heading) * globalDelta.y,
            globalDelta.heading
        );

        return new Pose2d (
            now.x + Math.cos(now.heading) * relDelta.x - Math.sin(now.heading) * relDelta.y,
            now.y + Math.sin(now.heading) * relDelta.x + Math.cos(now.heading) * relDelta.y,
            now.heading + relDelta.heading
        );
    }

    private void clipPoseToField(Pose2d pose) {
        pose.x = Utils.minMaxClip(pose.x, -72 + 6.2, 72 - 6.2);
        pose.y = Utils.minMaxClip(pose.y, -72 + 6.2, 72 - 6.2);
    }

    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
        pinpoint.setPosition(new Pose2D (DistanceUnit.INCH, pose.x, pose.y, AngleUnit.RADIANS, pose.heading));
        lastPinpointPose = lastPinpointMergePose = pose.clone();
    }

    public double getInstantaneousAngularVel () { return poseHistory.size() >= 2 ? (poseHistory.get(0).heading - poseHistory.get(1).heading) / (nanoTimes.get(0) - nanoTimes.get(1)) : 0; }

    public void relocalizeWithVision() {

        estimatedCameraPose = drivetrain.vision.update();
        if (estimatedCameraPose != null) {
            setPoseEstimate(estimatedCameraPose);
        }
    }

    public void updateField() {
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" distance", distanceTraveled);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" velocity", relCurrentVel);

        TelemetryUtil.packet.put("Pinpoint x", pinpoint.getPosX());
        TelemetryUtil.packet.put("Pinpoint y", pinpoint.getPosY());
        TelemetryUtil.packet.put("Pinpoint heading", pinpoint.getHeading());

        TelemetryUtil.packet.put("Vision x", estimatedCameraPose != null ? estimatedCameraPose.x : "na");
        TelemetryUtil.packet.put("Vision y", estimatedCameraPose != null ? estimatedCameraPose.y : "na");
        TelemetryUtil.packet.put("Vision heading", estimatedCameraPose != null ? estimatedCameraPose.heading : "na");

        TelemetryUtil.packet.put("Vision : useCamera", useCamera);
        TelemetryUtil.packet.put("Vision : relocalize count", numberOfTimesRelocalizedWithCamera);

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, this.color); // blue
        DashboardUtil.drawRobot(fieldOverlay, lastPinpointPose, this.expectedColor);
        //DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
    }
}
