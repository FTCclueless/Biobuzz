package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
public class PinpointLocalizer {
    public static final String DEVICE_NAME = "pinpoint";
    public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static double X_POD_OFFSET = 3.391;
    public static double Y_POD_OFFSET = 0.582;
    public static double ACCEL_FILTER = 0.3;

    private final GoBildaPinpointDriver pinpoint;

    private Pose2d pose = new Pose2d(0, 0, 0);
    private Pose2d globalVelocity = new Pose2d(0, 0, 0);
    private Pose2d relativeVelocity = new Pose2d(0, 0, 0);
    private Pose2d globalAccel = new Pose2d(0, 0, 0);
    private Pose2d lastGlobalVelocity = null;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);
        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
        if (Globals.RUNMODE != RunMode.TELEOP) pinpoint.resetPosAndIMU();
        update(0);
    }

    public void update(double dt) {
        pinpoint.update();

        double heading = pinpoint.getHeading();
        pose = new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), heading);

        double vx = pinpoint.getVelX(DistanceUnit.INCH);
        double vy = pinpoint.getVelY(DistanceUnit.INCH);
        double w = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        globalVelocity = new Pose2d(vx, vy, w);

        double c = Math.cos(heading), s = Math.sin(heading);
        relativeVelocity = new Pose2d(vx * c + vy * s, -vx * s + vy * c, w);

        if (lastGlobalVelocity != null && dt > 0) {
            double ax = (vx - lastGlobalVelocity.x) / dt;
            double ay = (vy - lastGlobalVelocity.y) / dt;
            double aw = (w - lastGlobalVelocity.heading) / dt;
            globalAccel = new Pose2d(
                    globalAccel.x + (ax - globalAccel.x) * ACCEL_FILTER,
                    globalAccel.y + (ay - globalAccel.y) * ACCEL_FILTER,
                    globalAccel.heading + (aw - globalAccel.heading) * ACCEL_FILTER);
        }
        lastGlobalVelocity = globalVelocity;
    }

    public void setPose(Pose2d p) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading));
        pose = p.clone();
        lastGlobalVelocity = null;
        globalAccel = new Pose2d(0, 0, 0);
    }

    public Pose2d getPose() { return pose; }

    public Pose2d getGlobalVelocity() { return globalVelocity; }

    public Pose2d getRelativeVelocity() { return relativeVelocity; }

    public Pose2d getGlobalAccel() { return globalAccel; }

    public GoBildaPinpointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }

    public double getFrequency() { return pinpoint.getFrequency(); }
}
