package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Turret Calibrator", group = "test")
public class TurretCalibrator extends LinearOpMode {
    public static double SWEEP_NEGATIVE_DEG = -120;

    public static double SWEEP_POSITIVE_DEG = 30;
    public static double STEP_DEG = 15;

    public static double PROBE = 0.02;

    public static long SETTLE_MS = 600;

    private Robot robot;
    private Turret turret;
    private double zeroPos;

    static final class Point {
        final double position, angle;
        final int approach;
        Point(double position, double angle, int approach) {
            this.position = position;
            this.angle = angle;
            this.approach = approach;
        }
    }

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        turret = robot.shooter.turret;
        if (!turret.hasEncoder()) {
            telemetry.addLine("Turret Calibrator needs the turret encoder.");
            telemetry.addLine("Set Turret.ENCODER_PORT to the motor whose encoder port it uses.");
            telemetry.update();
            waitForStart();
            return;
        }

        double p = Turret.ZERO_POSITION;
        boolean zeroSet = false;
        boolean lastA = false, lastL = false, lastR = false, lastLB = false, lastRB = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_left && !lastL) p -= 0.002;
            if (gamepad1.dpad_right && !lastR) p += 0.002;
            if (gamepad1.left_bumper && !lastLB) p -= 0.01;
            if (gamepad1.right_bumper && !lastRB) p += 0.01;
            p = Math.max(0, Math.min(1, p));
            if (gamepad1.a && !lastA) {
                tick(p);
                turret.zeroEncoderHere();
                zeroPos = p;
                zeroSet = true;
            }
            lastA = gamepad1.a; lastL = gamepad1.dpad_left; lastR = gamepad1.dpad_right;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;
            tick(p);

            telemetry.addLine("Turret Calibrator");
            telemetry.addLine("dpad L/R: fine nudge   bumpers: coarse nudge");
            telemetry.addLine("Point the turret EXACTLY straight ahead, then press A.");
            telemetry.addData("servo position", "%.4f", p);
            telemetry.addData("encoder angle (deg)", "%.2f", Math.toDegrees(turret.getEncoderAngle()));
            telemetry.addData("zero set", zeroSet ? String.format(Locale.US, "yes, at %.4f", zeroPos)
                    : "NO -- press A before start");
            telemetry.update();
        }
        if (isStopRequested()) return;
        if (!zeroSet) {
            hold(new String[]{"Zero was never set. Restart, line the turret up, press A."}, p);
            return;
        }

        List<Point> points = new ArrayList<>();
        points.add(measure(zeroPos, 0));

        Point probe = measure(zeroPos + PROBE, 0);
        double slope = (probe.angle - points.get(0).angle) / PROBE;
        if (Math.abs(Math.toDegrees(slope * PROBE)) < 0.5) {
            hold(new String[]{"PROBLEM: a probe move barely turned the turret.",
                    "Check the servos are powered and TICKS_PER_TURRET_REV is right,",
                    "or raise PROBE."}, zeroPos);
            return;
        }
        points.add(probe);
        measure(zeroPos, 0);

        List<Double> negPositions = new ArrayList<>();
        for (double deg = -STEP_DEG; deg >= SWEEP_NEGATIVE_DEG - 1e-9 && opModeIsActive(); deg -= STEP_DEG) {
            double[] fit = fit(points, zeroPos, slope);
            double pos = zeroPos + Math.toRadians(deg) / fit[0];
            if (pos < 0 || pos > 1) break;
            Point pt = measure(pos, -1);
            if (pastLimit(pt)) { measure(negPositions.isEmpty() ? zeroPos : last(negPositions), -1); break; }
            points.add(pt);
            negPositions.add(pos);
        }
        for (int i = negPositions.size() - 2; i >= 0 && opModeIsActive(); i--) {
            points.add(measure(negPositions.get(i), +1));
        }
        points.add(measure(zeroPos, +1));

        double[] fit = fit(points, zeroPos, slope);
        for (double deg = STEP_DEG; deg <= SWEEP_POSITIVE_DEG + 1e-9 && opModeIsActive(); deg += STEP_DEG) {
            double pos = zeroPos + Math.toRadians(deg) / fit[0];
            if (pos < 0 || pos > 1) break;
            Point pt = measure(pos, +1);
            if (pastLimit(pt)) break;
            points.add(pt);
        }
        measure(zeroPos, -1);
        if (!opModeIsActive()) return;

        fit = fit(points, zeroPos, slope);
        report(points, fit);
    }

    private static double last(List<Double> l) { return l.get(l.size() - 1); }

    private boolean pastLimit(Point pt) {
        double guard = Math.toRadians(2);
        return pt.angle < Turret.MIN_ANGLE + Turret.LIMIT_MARGIN + guard
                || pt.angle > Turret.MAX_ANGLE - Turret.LIMIT_MARGIN - guard;
    }

    private Point measure(double pos, int approach) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < SETTLE_MS) {
            tick(pos);
            telemetry.addData("servo position", "%.4f", pos);
            telemetry.addData("encoder angle (deg)", "%.2f", Math.toDegrees(turret.getEncoderAngle()));
            telemetry.update();
        }
        return new Point(pos, turret.getEncoderAngle(), approach);
    }

    static double[] fit(List<Point> pts, double zeroPos, double fallbackSlope) {
        int n = pts.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (Point p : pts) {
            double x = p.position - zeroPos;
            sx += x; sy += p.angle; sxx += x * x; sxy += x * p.angle;
        }
        double den = n * sxx - sx * sx;
        if (n < 2 || Math.abs(den) < 1e-12) return new double[]{fallbackSlope, 0, Double.NaN};
        double slope = (n * sxy - sx * sy) / den;
        double offset = (sy - slope * sx) / n;
        double ss = 0;
        for (Point p : pts) {
            double r = p.angle - (slope * (p.position - zeroPos) + offset);
            ss += r * r;
        }
        return new double[]{slope, offset, Math.sqrt(ss / n)};
    }

    static double backlash(List<Point> pts) {
        double sum = 0;
        int n = 0;
        for (Point a : pts) {
            if (a.approach != -1) continue;
            for (Point b : pts) {
                if (b.approach == +1 && Math.abs(a.position - b.position) < 1e-9) {
                    sum += Math.abs(a.angle - b.angle);
                    n++;
                    break;
                }
            }
        }
        return n > 0 ? sum / n : Double.NaN;
    }

    private void report(List<Point> points, double[] fit) {
        double slope = fit[0], offset = fit[1], rms = fit[2];
        double positionPerRadian = 1.0 / slope;

        double zero = zeroPos - offset / slope;
        double posAtMin = zero + Turret.MIN_ANGLE * positionPerRadian;
        double posAtMax = zero + Turret.MAX_ANGLE * positionPerRadian;
        double lash = backlash(points);

        List<String> lines = new ArrayList<>();
        lines.add(String.format(Locale.US, "points: %d   fit residual: %.2f deg", points.size(),
                Math.toDegrees(rms)));
        lines.add(String.format(Locale.US, "backlash: %.2f deg", Math.toDegrees(lash)));
        lines.add(String.format(Locale.US, "servo position at -310 / +50 deg: %.3f / %.3f",
                posAtMin, posAtMax));
        lines.add("");
        lines.add("=== paste into Turret ===");
        lines.add(String.format(Locale.US, "ZERO_POSITION = %.4f;", zero));
        lines.add(String.format(Locale.US, "POSITION_PER_RADIAN = %.5f;", positionPerRadian));
        lines.add("");
        if (posAtMin < 0 || posAtMin > 1 || posAtMax < 0 || posAtMax > 1) {
            lines.add("WARNING: the full -310..+50 window does not fit in the servos' travel.");
            lines.add("  The turret cannot reach its whole range with this gearing.");
        }
        if (Math.toDegrees(rms) > 1.5) {
            lines.add("WARNING: points do not lie on a straight line. Check for slipping");
            lines.add("  gears/belt, or TICKS_PER_TURRET_REV being wrong.");
        }
        if (Math.toDegrees(lash) > Math.toDegrees(Turret.MAX_TRIM)) {
            lines.add("WARNING: backlash exceeds MAX_TRIM, so trim cannot fully correct it.");
            lines.add("  Tighten the gear mesh, or raise MAX_TRIM.");
        } else if (!Double.isNaN(lash)) {
            lines.add("Backlash is within what encoder trim corrects. Good.");
        }
        hold(lines.toArray(new String[0]), zeroPos);
    }

    private void hold(String[] lines, double pos) {
        while (opModeIsActive()) {
            tick(pos);
            for (String l : lines) telemetry.addLine(l);
            telemetry.update();
        }
    }

    private void tick(double pos) {
        robot.sensors.update();
        turret.setManualPosition(pos);
        turret.update();
        robot.hardwareQueue.update();
        TelemetryUtil.sendTelemetry();
    }
}
