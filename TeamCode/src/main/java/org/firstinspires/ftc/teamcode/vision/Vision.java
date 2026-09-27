package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class Vision {

    public ArrayList<AprilTagDetection> detections = null;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal = null;

    public long frameAcquisitionNanoTime = 0;
    public static double decisionMarginLimit = 55;

    public Vision (HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public void init(HardwareMap HardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setNumThreads(3) //optimize this
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) // 640x480: 549.651, 549.651, 317.108, 236.644; 320x240: 281.5573273, 281.366942, 156.3332591, 119.8965271
                .setCameraPose(
                        new Position(DistanceUnit.MM, -135.67, -143.8, 228, 0),//y is forward backwards x is sideways
                        new YawPitchRollAngles(AngleUnit.DEGREES, 180, -60, 0, 0))
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(HardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        TelemetryUtil.dashboard.startCameraStream(visionPortal, 30);

        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS); //tune this

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(100);
        } catch (Exception e) {
            Log.i("Vision", "Camera Exposure/Gain Control got fried \n" + e);
        }

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public Pose2d update() {
        detections = aprilTagProcessor.getFreshDetections();

        if (detections != null && !detections.isEmpty()) {

            ArrayList<AprilTagDetection> positionTagDetections = new ArrayList<>();
            for (AprilTagDetection detection: detections) {
                if (detection.id == 20 || detection.id == 24) {
                    positionTagDetections.add(detection);
                }
            }

            if (positionTagDetections.size() >= 2 && Globals.fullField) {
                AprilTagDetection detection1 = positionTagDetections.get(0);
                AprilTagDetection detection2 = positionTagDetections.get(1);

                frameAcquisitionNanoTime = detection1.frameAcquisitionNanoTime;

                Pose3D robotPose1 = detection1.robotPose;
                Pose3D robotPose2 = detection2.robotPose;

                if (robotPose1 != null && robotPose2 != null) {
                    Pose2d botPose1 = Pose2d.from3D(robotPose1);
                    Pose2d botPose2 = Pose2d.from3D(robotPose2);

                    double d = detection1.decisionMargin / (detection2.decisionMargin * 2);

                    Pose2d p = new Pose2d(botPose1.x * d + botPose2.x * d, botPose1.y * d + botPose2.y * d, detection1.decisionMargin >= detection2.decisionMargin ? botPose1.heading : botPose2.heading);

                    p.heading += Math.PI / 2;
                    return p;
                }
            } else if (!positionTagDetections.isEmpty()) {
                AprilTagDetection detection = positionTagDetections.get(0);

                TelemetryUtil.packet.put("Vision : Decision Margin", String.valueOf(detection.decisionMargin));
                if (detection.decisionMargin < decisionMarginLimit) return null;

                frameAcquisitionNanoTime = detection.frameAcquisitionNanoTime;

                Pose3D robotPose = detection.robotPose;

                if (robotPose != null) {
                    Pose2d p = Pose2d.from3D(robotPose);

                    p.heading += Math.PI / 2;
                    return p;
                }
            }
        }

        return null;
    }
}
