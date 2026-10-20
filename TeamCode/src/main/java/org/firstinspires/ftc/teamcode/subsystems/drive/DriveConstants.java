package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.utils.PID;

@Config
public class DriveConstants {
    public static double V_FORWARD = 81.175;

    public static double V_STRAFE = 66.8431;

    public static double A_FORWARD = 70.0;

    public static double A_STRAFE = 52.0;

    public static double STALL_RATIO = 3.5;

    public static double SAFETY = 0.85;

    public static double TRACK_WIDTH = 14.0;

    public static double WHEEL_BASE = 13.0;

    public static double MAX_WHEEL_SPEED = 85.0;

    public static double CONTOUR_GAIN = 3.2;

    public static double LAG_GAIN = 0.9;

    public static double HEADING_GAIN = 4.0;

    public static double ACCEL_LEAD = 0.03;

    public static double FORWARD_GAIN = 1.0;

    public static double STRAFE_GAIN = 1.25;

    public static double GVF_CONTOUR_GAIN = 0.9;

    public static double GVF_LAG_GAIN = 0.25;

    public static double START_VELOCITY_FLOOR = 6.0;

    public static double POSITION_TOLERANCE = 1.0;
    public static double STATIC_POWER = 0.06;
    public static double PATH_TIMEOUT_FACTOR = 2.0;
    public static double PATH_TIMEOUT_EXTRA = 1.5;

    public static PID xPID = new PID(0.2, 0.0, 0.007);
    public static PID yPID = new PID(0.2, 0.0, 0.007);
    public static PID turnPID = new PID(0.4, 0.0, 0.002);
    public static PID hPID = new PID(0.53, 0.0, 0.0);

    public static double TURN_K_STATIC = 0.15;
    public static double X_THRESH = 1.5;
    public static double Y_THRESH = 1.5;
    public static double H_THRESH = Math.toRadians(2.5);

    public static double WAYPOINT_THRESH = 3.0;

    public static FrictionEllipse ellipse() {
        return new FrictionEllipse(V_FORWARD, V_STRAFE, A_FORWARD, A_STRAFE,
                MAX_WHEEL_SPEED, (TRACK_WIDTH + WHEEL_BASE) / 2.0, STALL_RATIO);
    }

    public static MecanumKinematics kinematics() {
        return new MecanumKinematics(TRACK_WIDTH, WHEEL_BASE, MAX_WHEEL_SPEED);
    }

    public static PathBuilder pathBuilder() {
        return new PathBuilder().ellipse(ellipse()).safetyFactor(SAFETY);
    }

    public static PathFollower follower(Trajectory trajectory, MecanumKinematics kinematics) {
        PathFollower f = new PathFollower(trajectory, kinematics)
                .headingGain(HEADING_GAIN)
                .contourGain(CONTOUR_GAIN)
                .lagGain(LAG_GAIN)
                .accelLead(ACCEL_LEAD)
                .forwardGain(FORWARD_GAIN)
                .strafeGain(STRAFE_GAIN)
                .startVelocityFloor(START_VELOCITY_FLOOR)
                .positionTolerance(POSITION_TOLERANCE)
                .staticPower(STATIC_POWER);
        f.gvf().contourGain(GVF_CONTOUR_GAIN).lagGain(GVF_LAG_GAIN);
        return f;
    }
}
