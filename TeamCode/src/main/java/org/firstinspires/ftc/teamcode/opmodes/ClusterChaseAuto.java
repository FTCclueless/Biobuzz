package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.BiobuzzField;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.BallClusterer;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

/**
 * Drives to the biggest visible cluster of POLLEN / own NECTAR and sweeps through it with the intake.
 *
 *   SEARCH   no cluster tracked: turn in place slowly until the Limelight sees one
 *   APPROACH path to a point APPROACH_STANDOFF short of the cluster, facing it. Re-plans if the
 *            cluster estimate moves (it gets more accurate as we get closer), until COMMIT_DIST
 *   SWEEP    intake on, straight line through the cluster and SWEEP_PAST beyond it
 *   -> forget that cluster, back to SEARCH, until MAX_CLUSTERS or the time limit
 *
 * Init: gamepad1 X = blue, B = red (decides which NECTAR counts as ours).
 */
@Config
@Autonomous(name = "Cluster Chase Auto", group = "Auto")
public class ClusterChaseAuto extends LinearOpMode {
    public static double AUTO_SECONDS = 28;
    public static int MAX_CLUSTERS = 3;

    /** Add to the facing-the-cluster heading. 0 = intake on the front (same side as the Limelight). */
    public static double INTAKE_HEADING_OFFSET_DEG = 0;
    public static double RETARGET_DIST = 4.0;   // in: re-plan if the approach point moved this much
    public static double COMMIT_DIST = 18.0;    // in: closer than this, stop re-planning
    public static double SWEEP_PAST = 6.0;      // in beyond the far edge of the cluster
    public static long SWEEP_TIMEOUT_MS = 2500;
    public static long APPROACH_TIMEOUT_MS = 4000;
    public static double SEARCH_TURN = 0.25;    // turn power while searching (+ = CCW)
    public static long SEARCH_TIMEOUT_MS = 6000;

    private enum State { SEARCH, APPROACH, SWEEP, DONE }

    private Robot robot;
    private BallDetection balls;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        balls = new BallDetection(hardwareMap);
        balls.start();

        boolean red = Globals.isRed;
        Boolean poseSetFor = null;
        while (opModeInInit()) {
            if (gamepad1.x) red = false;
            if (gamepad1.b) red = true;
            if (poseSetFor == null || poseSetFor != red) {
                Globals.isRed = red;
                BiobuzzField.setSide(red ? BiobuzzField.Side.RED : BiobuzzField.Side.BLUE);
                robot.drivetrain.setPoseEstimate(BiobuzzField.field().start());
                poseSetFor = red;
            }

            balls.update();
            robot.update();
            telemetry.addData("Alliance (X blue / B red)", red ? "RED" : "BLUE");
            telemetry.addData("Limelight connected", balls.getLimelight().isConnected());
            telemetry.addData("Balls tracked", balls.getTrackedBalls().size());
            telemetry.update();
        }
        if (isStopRequested()) { balls.stop(); return; }

        LogUtil.init();
        long autoStart = System.currentTimeMillis();
        State state = State.SEARCH;
        long stateStart = autoStart;
        int collected = 0;
        Vector2 plannedApproach = null;
        BallClusterer.Target sweeping = null;

        while (opModeIsActive() && state != State.DONE) {
            long now = System.currentTimeMillis();
            if (now - autoStart > AUTO_SECONDS * 1000) break;

            balls.update();
            BallClusterer.Target target = balls.getTarget();
            Pose2d pose = Globals.ROBOT_POSITION;

            switch (state) {
                case SEARCH:
                    if (target != null) {
                        robot.drivetrain.stop();
                        plannedApproach = planApproach(target);
                        state = State.APPROACH;
                        stateStart = now;
                    } else if (now - stateStart > SEARCH_TIMEOUT_MS) {
                        state = State.DONE;
                    } else {
                        robot.drivetrain.follower.manual(0, 0, SEARCH_TURN);
                    }
                    break;

                case APPROACH:
                    if (target != null) {
                        double toApproach = pose.toVec2().minus(plannedApproach).mag();
                        double moved = Vector2.distance(target.approach, plannedApproach);
                        if (toApproach > COMMIT_DIST && moved > RETARGET_DIST) {
                            plannedApproach = planApproach(target);
                        }
                        sweeping = target;
                    }
                    if (!robot.drivetrain.isBusy() || now - stateStart > APPROACH_TIMEOUT_MS) {
                        if (sweeping == null) { state = State.SEARCH; stateStart = now; break; }
                        startSweep(sweeping);
                        state = State.SWEEP;
                        stateStart = now;
                    }
                    break;

                case SWEEP:
                    if (!robot.drivetrain.isBusy() || now - stateStart > SWEEP_TIMEOUT_MS) {
                        robot.intake.reqOff(true);
                        balls.markCollected(sweeping.cluster.centroid, sweeping.cluster.radius + 4);
                        collected++;
                        sweeping = null;
                        state = collected >= MAX_CLUSTERS ? State.DONE : State.SEARCH;
                        stateStart = now;
                    }
                    break;

                default:
                    break;
            }

            TelemetryUtil.packet.put("ClusterChase : state", state.toString());
            TelemetryUtil.packet.put("ClusterChase : collected", collected);
            robot.update();
        }

        robot.drivetrain.stop();
        robot.intake.reqOff(true);
        robot.update();
        balls.stop();
        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
    }

    private double driveHeading(double faceCluster) {
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
