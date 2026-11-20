package org.firstinspires.ftc.teamcode.vision;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

/**
 * Drives to the best visible cluster of POLLEN / own NECTAR and sweeps through it with the intake.
 * Needs a Robot built with vision (new Robot(hardwareMap, true)) so robot.balls updates every loop.
 *
 *   SEARCH   no cluster tracked: turn in place until the Limelight sees one
 *   APPROACH path toward a point APPROACH_STANDOFF short of the cluster, facing it. Re-plans while
 *            the estimate moves (it sharpens as we close in), until COMMIT_DIST
 *   SWEEP    starts HANDOFF_DIST before the approach point, without stopping: intake on, straight
 *            through the cluster and SWEEP_PAST beyond it
 *   -> forget that cluster, back to SEARCH, until maxClusters or the time budget
 */
@Config
public class ClusterChaser {
    /** Add to the facing-the-cluster heading. 0 = intake on the front (same side as the Limelight). */
    public static double INTAKE_HEADING_OFFSET_DEG = 0;
    public static double RETARGET_DIST = 4.0;   // in: re-plan if the approach point moved this much
    public static double COMMIT_DIST = 18.0;    // in: closer than this, stop re-planning
    /** in: switch to the sweep this far before the approach point, so the robot never stops there. */
    public static double HANDOFF_DIST = 6.0;
    public static double SWEEP_PAST = 6.0;      // in beyond the far edge of the cluster
    public static long SWEEP_TIMEOUT_MS = 2500;
    public static long APPROACH_TIMEOUT_MS = 4000;
    public static double SEARCH_TURN = 0.25;    // turn power while searching (+ = CCW)
    public static long SEARCH_TIMEOUT_MS = 6000;

    public enum State { SEARCH, APPROACH, SWEEP, DONE }

    private final Robot robot;
    private final BallDetection balls;

    public State state = State.DONE;
    public int collected = 0;
    private int maxClusters;
    private long endAt, stateStart;
    private Vector2 plannedApproach;
    private BallClusterer.Target sweeping;

    public ClusterChaser(Robot robot) {
        if (robot.balls == null) throw new IllegalStateException("ClusterChaser needs new Robot(hardwareMap, true)");
        this.robot = robot;
        this.balls = robot.balls;
    }

    /** Blocking: chase up to maxClusters within budgetMs, then stop the drive and intake. Returns clusters swept. */
    public int run(int maxClusters, long budgetMs) {
        start(maxClusters, budgetMs);
        robot.waitWhile(this::update);
        finish();
        return collected;
    }

    public void start(int maxClusters, long budgetMs) {
        this.maxClusters = maxClusters;
        long now = System.currentTimeMillis();
        endAt = now + budgetMs;
        collected = 0;
        sweeping = null;
        setState(State.SEARCH, now);
    }

    /** One step; call once per loop after robot.update(). Returns false when done. */
    public boolean update() {
        long now = System.currentTimeMillis();
        if (now > endAt) setState(State.DONE, now);

        BallClusterer.Target target = balls.getTarget();
        Pose2d pose = Globals.ROBOT_POSITION;

        switch (state) {
            case SEARCH:
                if (target != null) {
                    robot.drivetrain.stop(); // leave manual turning before following a path
                    plannedApproach = planApproach(target);
                    sweeping = target;
                    setState(State.APPROACH, now);
                } else if (now - stateStart > SEARCH_TIMEOUT_MS) {
                    setState(State.DONE, now);
                } else {
                    robot.drivetrain.follower.manual(0, 0, SEARCH_TURN);
                }
                break;

            case APPROACH: {
                if (target != null) {
                    double moved = Vector2.distance(target.approach, plannedApproach);
                    double toApproach = pose.toVec2().minus(plannedApproach).mag();
                    if (toApproach > COMMIT_DIST && moved > RETARGET_DIST) plannedApproach = planApproach(target);
                    sweeping = target;
                }
                double toApproach = pose.toVec2().minus(plannedApproach).mag();
                if (toApproach < HANDOFF_DIST || !robot.drivetrain.isBusy() || now - stateStart > APPROACH_TIMEOUT_MS) {
                    startSweep(sweeping);
                    setState(State.SWEEP, now);
                }
                break;
            }

            case SWEEP:
                if (!robot.drivetrain.isBusy() || now - stateStart > SWEEP_TIMEOUT_MS) {
                    balls.markCollected(sweeping.cluster.centroid, sweeping.cluster.radius + 4);
                    collected++;
                    sweeping = null;
                    // Intake stays on into the next approach: balls knocked loose still get picked up.
                    setState(collected >= maxClusters ? State.DONE : State.SEARCH, now);
                }
                break;

            default:
                break;
        }

        TelemetryUtil.packet.put("ClusterChase : state", state.toString());
        TelemetryUtil.packet.put("ClusterChase : collected", collected);
        return state != State.DONE;
    }

    public void finish() {
        robot.drivetrain.stop();
        robot.intake.reqOff(true);
        robot.update();
    }

    private void setState(State s, long now) {
        state = s;
        stateStart = now;
    }

    private static double driveHeading(double faceCluster) {
        return faceCluster + Math.toRadians(INTAKE_HEADING_OFFSET_DEG);
    }

    private Vector2 planApproach(BallClusterer.Target t) {
        Pose2d p = Globals.ROBOT_POSITION;
        robot.drivetrain.follow(line(new Pose(p.x, p.y), new Pose(t.approach.x, t.approach.y))
                .constant(driveHeading(t.heading)));
        return t.approach;
    }

    private void startSweep(BallClusterer.Target t) {
        Pose2d p = Globals.ROBOT_POSITION;
        Vector2 dir = t.cluster.centroid.minus(p.toVec2());
        double d = dir.mag();
        dir = d > 1e-6 ? dir.div(d) : new Vector2(Math.cos(t.heading), Math.sin(t.heading));
        Vector2 end = t.cluster.centroid.plus(dir.times(t.cluster.radius + SWEEP_PAST));
        robot.intake.reqIntake(true);
        robot.drivetrain.follow(line(new Pose(p.x, p.y), new Pose(end.x, end.y))
                .constant(driveHeading(Math.atan2(dir.y, dir.x))));
    }
}
