package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;

/**
 * Body of the simple autos. PARK: leave the start and drive into the LOADING ZONE (LEAVE + PARK).
 * SHOOT: leave toward the centre line, then auto-aim and fire the preloads.
 * CORNER_PARK: start in the corner beside the LOADING ZONE, drive into it (LEAVE + PARK).
 * The Red/Blue subclasses only pick the alliance (which teleop then inherits) and the mode.
 *
 * Placement, every auto: the robot's RIGHT side flat on the alliance wall. Red faces +x (heading 0),
 * blue faces -x (heading 180). Mirrored like that, the shot stays inside the turret's window
 * (-175..+55 deg): at heading 0 the blue shot needs +67 deg, past the clockwise limit.
 */
@Config
public abstract class SimpleAuto extends LinearOpMode {
    /** Shoot autos: how far to drive from the start wall toward the centre line before shooting (in). */
    public static double LEAVE_DISTANCE = 12;
    /** Wait after START, e.g. to stay out of an alliance partner's way. */
    public static long START_DELAY_MS = 0;
    /** A path that hasn't finished by now is abandoned: a robot pinned on a wall never finishes. */
    public static long FOLLOW_TIMEOUT_MS = 5000;
    /** Give the flywheel this long to read READY, then fire at whatever speed it reached. */
    public static long READY_TIMEOUT_MS = 3000;
    public static long SHOOT_MS = 1500;
    /** Every park drives this much past its target, so a robot stopping just short still counts. */
    public static double PARK_EXTRA_IN = 1.0;

    public enum Mode { PARK, SHOOT, CORNER_PARK }

    protected abstract BiobuzzField.Side side();
    protected abstract Mode mode();

    private Robot robot;

    /**
     * Right side on the alliance wall (centre half a robot width off it), red facing +x, blue facing -x.
     * Middle of the wall (in front of the hive), or the corner at the drivers' left, beside the LOADING ZONE,
     * with the back on the side wall (red -x, blue +x). There the robot ends 6 in short of the zone (G304.E).
     */
    static Pose2d startPose(boolean red, boolean corner) {
        double sign = red ? 1 : -1;
        double y = -sign * (72 - Globals.ROBOT_WIDTH / 2);
        double x = corner ? -sign * (72 - Globals.ROBOT_LENGTH / 2) : 0;
        return new Pose2d(x, y, red ? 0 : Math.PI);
    }

    /** to, moved a further extra inches along the from -> to direction. */
    static Pose overshoot(Pose from, Pose to, double extra) {
        double dx = to.x() - from.x(), dy = to.y() - from.y(), len = Math.hypot(dx, dy);
        if (len < 1e-9) return to;
        return new Pose(to.x() + dx / len * extra, to.y() + dy / len * extra, to.heading());
    }

    /** Start pose moved distance toward y = 0 (the centre line), same x and heading. */
    static Pose leaveSpot(Pose2d start, double distance) {
        return new Pose(start.x, start.y - Math.signum(start.y) * distance, start.heading);
    }

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        BiobuzzField.setSide(side());

        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        boolean red = side() == BiobuzzField.Side.RED;
        Pose2d start = startPose(red, mode() == Mode.CORNER_PARK);
        robot.drivetrain.setPoseEstimate(start);
        Globals.AUTO_ENDING_POSE = start.clone(); // teleop starts from a real pose even if this is stopped early
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            telemetry.addData("Auto", "%s %s", side(), mode());
            telemetry.addData("Place", "right side on the %s wall, facing %s, %s", side(), red ? "+x" : "-x",
                    mode() == Mode.CORNER_PARK ? "loading-zone corner, back on the side wall" : "centred on the wall");
            telemetry.addData("Pose reset", robot.drivetrain.getPoseResetStatus());
            telemetry.update();
        }
        if (isStopRequested()) return;

        robot.waitFor(START_DELAY_MS);
        Pose from = Utils.fromPose2d(start);
        switch (mode()) {
            case SHOOT:
                robot.shooter.setManual(false);
                robot.shooter.reqAim(true); // spins up and tracks while driving
                drive(from, leaveSpot(start, LEAVE_DISTANCE), start.heading);
                fire();
                break;
            case PARK:
            case CORNER_PARK: // same target, the corner start is just closer to it
                drive(from, overshoot(from, BiobuzzField.field().park(), PARK_EXTRA_IN), start.heading);
                break;
        }

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        telemetry.addData("ended at", Globals.ROBOT_POSITION.toString());
        telemetry.update();
    }

    private void drive(Pose from, Pose to, double heading) {
        robot.drivetrain.follow(line(from, to).constant(heading));
        robot.waitWhileWithTimeout(robot.drivetrain::isBusy, FOLLOW_TIMEOUT_MS);
        if (robot.drivetrain.isBusy()) robot.drivetrain.stop(); // stop pushing; a finished path keeps holding
    }

    private void fire() {
        robot.waitWhileWithTimeout(() -> robot.shooter.state != Shooter.State.READY, READY_TIMEOUT_MS);
        // AIMING ignores reqShoot: out of reach of the table speed, fire at the speed it got to.
        if (robot.shooter.state == Shooter.State.AIMING) robot.shooter.state = Shooter.State.READY;
        robot.shooter.reqShoot(true);
        robot.waitFor(SHOOT_MS);
        robot.shooter.reqStop(true);
        robot.waitFor(100); // let the stop go through before the OpMode ends
    }
}
