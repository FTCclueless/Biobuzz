package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Drive Characterizer", group = "test")
public class DriveCharacterizer extends LinearOpMode {
    public static double POWER_LOW = 0.35;
    public static double POWER_HIGH = 0.70;
    public static double RUN_SECONDS = 1.4;
    public static double MAX_TRAVEL = 90.0;
    public static double SETTLE_FRACTION = 0.5;
    public static double TURN_POWER = 0.5;
    public static double TURN_SECONDS = 1.2;
    public static double REST_SECONDS = 1.0;

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        MergeLocalizer.useCamera = false;

        telemetry.addLine("Drive Characterizer");
        telemetry.addLine();
        telemetry.addLine("Needs a clear straight lane of about 8 feet ahead,");
        telemetry.addLine("and a couple of feet either side for the strafe runs.");
        telemetry.addLine("Runs: forward x2, strafe x2, turn x1.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();

        Run fwdLow  = straightRun("forward low",  POWER_LOW,  Axis.FORWARD);
        Run fwdHigh = straightRun("forward high", POWER_HIGH, Axis.FORWARD);
        Run strLow  = straightRun("strafe low",   POWER_LOW,  Axis.STRAFE);
        Run strHigh = straightRun("strafe high",  POWER_HIGH, Axis.STRAFE);
        Run turn    = turnRun();

        report(fwdLow, fwdHigh, strLow, strHigh, turn);
    }

    private enum Axis { FORWARD, STRAFE }

    private static final class Run {
        String name;
        double power;
        double steadyVelocity;
        double launchAccel;
        double timeConstant;
        double travel;
        boolean aborted;
    }

    private Run straightRun(String name, double power, Axis axis) {
        rest();
        Pose2d origin = ROBOT_POSITION.clone();
        double heading0 = origin.heading;

        List<Double> t = new ArrayList<>(), d = new ArrayList<>();
        long start = System.nanoTime();
        double travel = 0;
        boolean aborted = false;

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            if (elapsed > RUN_SECONDS) break;

            drive(axis, power);
            robot.hardwareQueue.update();
            robot.update();

            double dx = ROBOT_POSITION.x - origin.x;
            double dy = ROBOT_POSITION.y - origin.y;
            double along = axis == Axis.FORWARD
                    ?  dx * Math.cos(heading0) + dy * Math.sin(heading0)
                    : -dx * Math.sin(heading0) + dy * Math.cos(heading0);

            t.add(elapsed);
            d.add(along);
            travel = Math.abs(along);

            if (travel > MAX_TRAVEL) { aborted = true; break; }
        }

        stop_();
        Run r = summarize(name, power, t, d);
        r.travel = travel;
        r.aborted = aborted;
        return r;
    }

    private Run turnRun() {
        rest();
        List<Double> t = new ArrayList<>(), a = new ArrayList<>();
        long start = System.nanoTime();
        double last = ROBOT_POSITION.heading;
        double unwrapped = 0;

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            if (elapsed > TURN_SECONDS) break;

            robot.drivetrain.setMotorPowers(-TURN_POWER, -TURN_POWER, TURN_POWER, TURN_POWER);
            robot.hardwareQueue.update();
            robot.update();

            double h = ROBOT_POSITION.heading;
            double step = org.firstinspires.ftc.teamcode.utils.Utils.headingClip(h - last);
            unwrapped += step;
            last = h;

            t.add(elapsed);
            a.add(unwrapped);
        }

        stop_();
        Run r = summarize("turn", TURN_POWER, t, a);
        r.travel = Math.abs(unwrapped);
        return r;
    }

    private Run summarize(String name, double power, List<Double> t, List<Double> d) {
        Run r = new Run();
        r.name = name;
        r.power = power;
        if (t.size() < 8) return r;

        int settleFrom = (int) (t.size() * SETTLE_FRACTION);
        r.steadyVelocity = slope(t, d, settleFrom, t.size());

        int launchTo = Math.max(4, settleFrom / 2);
        r.launchAccel = 2.0 * quadCoefficient(t, d, 0, launchTo);

        double target = 0.632 * r.steadyVelocity;
        r.timeConstant = Double.NaN;
        int w = Math.max(3, t.size() / 12);
        for (int i = w; i < t.size(); i++) {
            double v = slope(t, d, i - w, i);
            if (Math.abs(v) >= Math.abs(target) && target != 0) {
                r.timeConstant = t.get(i);
                break;
            }
        }
        return r;
    }

    private static double slope(List<Double> t, List<Double> d, int from, int to) {
        int n = to - from;
        if (n < 2) return 0;
        double st = 0, sd = 0, stt = 0, std = 0;
        for (int i = from; i < to; i++) {
            double x = t.get(i), y = d.get(i);
            st += x; sd += y; stt += x * x; std += x * y;
        }
        double den = n * stt - st * st;
        if (Math.abs(den) < 1e-12) return 0;
        return (n * std - st * sd) / den;
    }

    private static double quadCoefficient(List<Double> t, List<Double> d, int from, int to) {
        double num = 0, den = 0;
        for (int i = from; i < to; i++) {
            double x2 = t.get(i) * t.get(i);
            num += x2 * d.get(i);
            den += x2 * x2;
        }
        return den < 1e-12 ? 0 : num / den;
    }

    private static double freeSpeed(Run low, Run high) {
        double dp = high.power - low.power;
        if (Math.abs(dp) < 1e-6) return Math.abs(high.steadyVelocity);
        double k = (Math.abs(high.steadyVelocity) - Math.abs(low.steadyVelocity)) / dp;
        double intercept = Math.abs(high.steadyVelocity) - k * high.power;
        return k * 1.0 + intercept;
    }

    private void drive(Axis axis, double p) {
        if (axis == Axis.FORWARD) {
            robot.drivetrain.setMotorPowers(p, p, p, p);
        } else {
            robot.drivetrain.setMotorPowers(-p, p, -p, p);
        }
    }

    private void stop_() {
        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
        robot.update();
    }

    private void rest() {
        long start = System.nanoTime();
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < REST_SECONDS) {
            robot.drivetrain.stopAllMotors();
            robot.hardwareQueue.update();
            robot.update();
        }
    }

    private void report(Run fwdLow, Run fwdHigh, Run strLow, Run strHigh, Run turn) {
        double vForward = freeSpeed(fwdLow, fwdHigh);
        double vStrafe = freeSpeed(strLow, strHigh);
        double aForward = Math.abs(fwdHigh.launchAccel);
        double aStrafe = Math.abs(strHigh.launchAccel);

        double maxWheelSpeed = vForward;
        double strafeGain = vStrafe > 1e-6 ? vForward / vStrafe : 1.0;

        double loop = robot.sensors.loopTime;
        double tauStrafe = Double.isNaN(strHigh.timeConstant) ? 0.15 : strHigh.timeConstant;
        double tauTurn = Double.isNaN(turn.timeConstant) ? 0.15 : turn.timeConstant;

        double contourStart = 0.5 / Math.max(0.05, tauStrafe + loop);
        double headingStart = 0.5 / Math.max(0.05, tauTurn + loop);
        double accelLead = Math.min(0.08, Math.max(0.02, loop + tauTurn * 0.25));

        while (opModeIsActive()) {
            telemetry.addLine("=== measured ===");
            line(fwdLow); line(fwdHigh); line(strLow); line(strHigh); line(turn);
            telemetry.addLine();
            telemetry.addData("loop time (s)", "%.4f", loop);
            telemetry.addLine();
            telemetry.addLine("=== paste into DriveConstants ===");
            telemetry.addLine(String.format(Locale.US, "V_FORWARD  = %.1f;", vForward));
            telemetry.addLine(String.format(Locale.US, "V_STRAFE   = %.1f;", vStrafe));
            telemetry.addLine(String.format(Locale.US, "A_FORWARD  = %.1f;   // achieved, not the traction limit -- use SlipTest", aForward));
            telemetry.addLine(String.format(Locale.US, "A_STRAFE   = %.1f;   // achieved, not the traction limit -- use SlipTest", aStrafe));
            telemetry.addLine(String.format(Locale.US, "MAX_WHEEL_SPEED = %.1f;", maxWheelSpeed));
            telemetry.addLine();
            telemetry.addLine("=== starting gains, then tune up ===");
            telemetry.addLine(String.format(Locale.US, "HEADING_GAIN = %.2f;", headingStart));
            telemetry.addLine(String.format(Locale.US, "CONTOUR_GAIN = %.2f;", contourStart));
            telemetry.addLine(String.format(Locale.US, "LAG_GAIN     = %.2f;", contourStart / 3.5));
            telemetry.addLine(String.format(Locale.US, "ACCEL_LEAD   = %.3f;", accelLead));
            telemetry.addLine(String.format(Locale.US, "strafeGain   = %.2f  (apply if > 1.15)", strafeGain));
            telemetry.update();
            robot.update();
        }
    }

    private void line(Run r) {
        telemetry.addData(r.name, String.format(Locale.US,
                "p=%.2f  v=%.1f  a=%.0f  tau=%.3f  moved=%.0f%s",
                r.power, r.steadyVelocity, r.launchAccel, r.timeConstant, r.travel,
                r.aborted ? "  ABORTED (hit MAX_TRAVEL)" : ""));
    }
}
