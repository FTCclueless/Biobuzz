package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;

/**
 * The one place for tuned drive/pathing numbers. Drivetrain, the tuning opmodes, and autos all
 * read from here, so a value measured on the tester is entered once and used everywhere.
 *
 * Where each number comes from:
 *   V_FORWARD, V_STRAFE, MAX_WHEEL_SPEED  -> Drive Characterizer
 *   A_FORWARD, A_STRAFE                   -> Slip Test (run once per axis)
 *   *_GAIN, ACCEL_LEAD                    -> Pathing Tuner / Gain Sweeper
 *
 * Dashboard edits reset on restart -- copy final values into the defaults below.
 */
@Config
public class DriveConstants {
    // Friction ellipse: speed (in/s) and traction (in/s^2) limits
    public static double V_FORWARD = 62.0;
    public static double V_STRAFE = 48.0;
    public static double A_FORWARD = 70.0;
    public static double A_STRAFE = 52.0;
    public static double STALL_RATIO = 3.5;
    public static double SAFETY = 0.85;

    // Kinematics (in). Wheel-to-wheel distances, not the frame size.
    public static double TRACK_WIDTH = 14.0;
    public static double WHEEL_BASE = 13.0;
    public static double MAX_WHEEL_SPEED = 66.0;

    // Follower gains
    public static double HEADING_GAIN = 4.0;
    public static double CONTOUR_GAIN = 3.2;
    public static double LAG_GAIN = 0.9;
    public static double ACCEL_LEAD = 0.03;

    public static FrictionEllipse ellipse() {
        return new FrictionEllipse(V_FORWARD, V_STRAFE, A_FORWARD, A_STRAFE,
                MAX_WHEEL_SPEED, (TRACK_WIDTH + WHEEL_BASE) / 2.0, STALL_RATIO);
    }

    public static MecanumKinematics kinematics() {
        return new MecanumKinematics(TRACK_WIDTH, WHEEL_BASE, MAX_WHEEL_SPEED);
    }

    /** Start every auto path here. A bare {@code new PathBuilder()} plans against generic defaults. */
    public static PathBuilder pathBuilder() {
        return new PathBuilder().ellipse(ellipse()).safetyFactor(SAFETY);
    }

    public static PathFollower follower(Trajectory trajectory, MecanumKinematics kinematics) {
        return new PathFollower(trajectory, kinematics)
                .headingGain(HEADING_GAIN)
                .contourGain(CONTOUR_GAIN)
                .lagGain(LAG_GAIN)
                .accelLead(ACCEL_LEAD);
    }
}
