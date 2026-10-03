package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.utils.PID;

/**
 * Every tuned drive number, in one place. Drivetrain, the tuning opmodes, and every auto read
 * from here, so a value measured on a tester is entered once and used everywhere.
 *
 * WHICH TESTER PRODUCES WHAT
 *   Drive Characterizer  -> V_FORWARD, V_STRAFE, MAX_WHEEL_SPEED, and starting gains
 *   Slip Test            -> A_FORWARD (STRAFE=false), A_STRAFE (STRAFE=true)
 *   Pathing Tuner        -> CONTOUR_GAIN, HEADING_GAIN, LAG_GAIN, ACCEL_LEAD, SAFETY
 *   Gain Sweeper         -> one of CONTOUR_GAIN / HEADING_GAIN / ACCEL_LEAD, swept automatically
 *   tape measure         -> TRACK_WIDTH, WHEEL_BASE
 *
 * Dashboard edits are lost on restart. Once a number is final, type it into the default here.
 *
 * Not here on purpose: localizer and vision-fusion tuning (nMergeLocalizer, RobotEKF) are a
 * separate problem with their own @Config panels, and flywheel constants live in Flywheel.
 */
@Config
public class DriveConstants {

    // ---------------------------------------------------------------- limits (the plan)
    /** Top speed driving straight forward, in/s. Drive Characterizer. */
    public static double V_FORWARD = 62.0;
    /** Top speed strafing sideways, in/s. Always lower than forward. Drive Characterizer. */
    public static double V_STRAFE = 48.0;
    /** Forward acceleration the tyres can hold, in/s^2. Slip Test with STRAFE=false. */
    public static double A_FORWARD = 70.0;
    /** Sideways acceleration the tyres can hold, in/s^2. Slip Test with STRAFE=true. */
    public static double A_STRAFE = 52.0;
    /** How much harder the motors push from a standstill than at top speed. 3-4 is typical. */
    public static double STALL_RATIO = 3.5;
    /** Fraction of the measured limits paths are planned against. Leaves room to correct. */
    public static double SAFETY = 0.85;

    // ---------------------------------------------------------------- geometry
    /** Left wheel centre to right wheel centre, in. */
    public static double TRACK_WIDTH = 14.0;
    /** Front wheel centre to back wheel centre, in. */
    public static double WHEEL_BASE = 13.0;
    /** Speed of one wheel at full power, in/s. Drive Characterizer. */
    public static double MAX_WHEEL_SPEED = 66.0;

    // ---------------------------------------------------------------- follower gains
    /** Pulls the robot back onto the line when it drifts sideways off the path. */
    public static double CONTOUR_GAIN = 3.2;
    /** Catches the robot up along the path when it falls behind the plan. */
    public static double LAG_GAIN = 0.9;
    /** Turns the robot back to the planned heading. */
    public static double HEADING_GAIN = 4.0;
    /** Seconds of planned acceleration added to the speed command, to fight motor lag. */
    public static double ACCEL_LEAD = 0.03;
    /** Scales forward commands. Leave at 1.0 and correct with the ellipse instead. */
    public static double FORWARD_GAIN = 1.0;
    /** Scales strafe commands; mecanum wheels slip sideways so this is normally > 1. */
    public static double STRAFE_GAIN = 1.25;
    /** Vector-field steering strength. Raise if the robot rejoins the path too lazily. */
    public static double GVF_CONTOUR_GAIN = 0.9;
    /** Vector-field catch-up strength along the path. */
    public static double GVF_LAG_GAIN = 0.25;
    /** Minimum speed for the first couple of inches, so the robot breaks static friction. */
    public static double START_VELOCITY_FLOOR = 6.0;
    /** How close to the end pose counts as arrived, in. */
    public static double POSITION_TOLERANCE = 1.0;

    // ---------------------------------------------------------------- point-to-point PID
    // Used by goToPoint(), not by trajectories. Separate from the follower gains above.
    public static PID xPID = new PID(0.2, 0.0, 0.007);
    public static PID yPID = new PID(0.2, 0.0, 0.007);
    public static PID turnPID = new PID(0.4, 0.0, 0.002);
    public static PID hPID = new PID(0.53, 0.0, 0.0);
    /** Constant power added to beat turning friction once outside H_THRESH. */
    public static double TURN_K_STATIC = 0.15;
    public static double X_THRESH = 1.5;
    public static double Y_THRESH = 1.5;
    public static double H_THRESH = Math.toRadians(2.5);
    /** Looser arrival window for waypoints the robot should drive through, not stop at. */
    public static double WAYPOINT_THRESH = 3.0;

    // ---------------------------------------------------------------- factories

    /** Speed and acceleration limits, as the planner wants them. */
    public static FrictionEllipse ellipse() {
        return new FrictionEllipse(V_FORWARD, V_STRAFE, A_FORWARD, A_STRAFE,
                MAX_WHEEL_SPEED, (TRACK_WIDTH + WHEEL_BASE) / 2.0, STALL_RATIO);
    }

    public static MecanumKinematics kinematics() {
        return new MecanumKinematics(TRACK_WIDTH, WHEEL_BASE, MAX_WHEEL_SPEED);
    }

    /**
     * Start every auto path with this. A bare {@code new PathBuilder()} silently plans against
     * generic defaults instead of your measured limits.
     */
    public static PathBuilder pathBuilder() {
        return new PathBuilder().ellipse(ellipse()).safetyFactor(SAFETY);
    }

    /** A follower with every gain above applied. */
    public static PathFollower follower(Trajectory trajectory, MecanumKinematics kinematics) {
        PathFollower f = new PathFollower(trajectory, kinematics)
                .headingGain(HEADING_GAIN)
                .contourGain(CONTOUR_GAIN)
                .lagGain(LAG_GAIN)
                .accelLead(ACCEL_LEAD)
                .forwardGain(FORWARD_GAIN)
                .strafeGain(STRAFE_GAIN)
                .startVelocityFloor(START_VELOCITY_FLOOR)
                .positionTolerance(POSITION_TOLERANCE);
        f.gvf().contourGain(GVF_CONTOUR_GAIN).lagGain(GVF_LAG_GAIN);
        return f;
    }
}
