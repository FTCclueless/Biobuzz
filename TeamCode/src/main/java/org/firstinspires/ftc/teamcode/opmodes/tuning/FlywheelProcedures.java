package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

final class FlywheelProcedures {
    private final LinearOpMode op;
    private final Robot robot;
    private final Flywheel fw;
    private final Telemetry telemetry;

    FlywheelProcedures(LinearOpMode op, Robot robot) {
        this.op = op;
        this.robot = robot;
        this.fw = robot.shooter.flywheel;
        this.telemetry = op.telemetry;
    }

    static final class Point {
        double power, volts, velocity;
    }

    static final class Model {
        double kV = Double.NaN, kS = Double.NaN, kA = Double.NaN, tau = Double.NaN;
        final List<Point> points = new ArrayList<>();

        String problem() {
            if (!(kV > 0)) return "kV invalid: need at least 2 steady points with rising speed";
            if (Double.isNaN(tau)) return "tau invalid: the wheel never settled -- raise HOLD_SECONDS";
            if (kS < 0) return "kS negative: low power levels never reached steady state";
            return null;
        }
    }

    static final class Step {
        String name;
        double from, to;
        double settleTime = Double.NaN;
        double overshoot;
        double steadyError = Double.NaN;

        double steadyNoise = Double.NaN;
        double peak;
    }

    Model characterize(double powerLow, double powerHigh, int steps, double holdSeconds,
                       double steadyFraction, double restSeconds, double kaPower) {
        Model m = new Model();
        for (int i = 0; i < steps && op.opModeIsActive(); i++) {
            double p = steps == 1 ? powerHigh
                    : powerLow + (powerHigh - powerLow) * i / (steps - 1.0);
            coast(restSeconds);
            Point pt = holdOpenLoop(p, holdSeconds, steadyFraction);
            if (pt != null) m.points.add(pt);
        }
        coast(restSeconds);
        m.tau = spinUpTimeConstant(kaPower, holdSeconds);

        double[] fit = fitLine(m.points);
        m.kV = fit[0];
        m.kS = fit[1];
        m.kA = m.kV * m.tau;
        openLoop(0);
        return m;
    }

    private Point holdOpenLoop(double power, double holdSeconds, double steadyFraction) {
        long start = System.nanoTime();
        double sumV = 0, sumVolts = 0;
        int n = 0;
        while (op.opModeIsActive()) {
            double t = (System.nanoTime() - start) / 1e9;
            if (t > holdSeconds) break;
            openLoop(power);
            if (t > holdSeconds * (1.0 - steadyFraction)) {
                sumV += fw.getRawVelocity();
                sumVolts += power * robot.sensors.getVoltage();
                n++;
            }
            status(String.format(Locale.US, "holding power %.2f  (%.1f / %.1f s)",
                    power, t, holdSeconds));
        }
        if (n == 0) return null;
        Point p = new Point();
        p.power = power;
        p.velocity = sumV / n;
        p.volts = sumVolts / n;
        return p;
    }

    private double spinUpTimeConstant(double power, double holdSeconds) {
        List<Double> t = new ArrayList<>(), v = new ArrayList<>();
        long start = System.nanoTime();
        while (op.opModeIsActive()) {
            double e = (System.nanoTime() - start) / 1e9;
            if (e > holdSeconds) break;
            openLoop(power);
            t.add(e);
            v.add(fw.getRawVelocity());
            status("measuring spin-up time constant (kA)");
        }
        if (t.size() < 8) return Double.NaN;
        int from = (int) (t.size() * 0.8);
        double finalV = 0;
        for (int i = from; i < t.size(); i++) finalV += v.get(i);
        finalV /= (t.size() - from);
        if (finalV < 1e-6) return Double.NaN;
        double target = 0.632 * finalV;
        for (int i = 0; i < t.size(); i++) if (v.get(i) >= target) return t.get(i);
        return Double.NaN;
    }

    static void apply(Model m) {
        Flywheel.kV = m.kV;
        Flywheel.kA = m.kA;
        Flywheel.kS = m.kS;
        Flywheel.FF_TABLE.clear();
        for (Point p : m.points) Flywheel.FF_TABLE.put(p.velocity, p.volts);
    }

    static List<String> pasteLines(Model m) {
        List<String> out = new ArrayList<>();
        out.add("=== paste into Flywheel ===");
        out.add(String.format(Locale.US, "kV = %.6f;", m.kV));
        out.add(String.format(Locale.US, "kA = %.6f;", m.kA));
        out.add(String.format(Locale.US, "kS = %.4f;", m.kS));
        out.add("=== paste into Flywheel FF_TABLE static block ===");
        for (Point p : m.points) {
            out.add(String.format(Locale.US, "FF_TABLE.put(%.1f, %.3f);", p.velocity, p.volts));
        }
        return out;
    }

    void holdClosedLoop(double target, double seconds) {
        long start = System.nanoTime();
        while (op.opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            closedLoop(target);
            status(String.format(Locale.US, "settling at %.0f in/s", target));
        }
    }

    Step step(String name, double from, double to, double holdSeconds, double settleBand) {
        Step r = new Step();
        r.name = name;
        r.from = from;
        r.to = to;
        double dir = Math.signum(to - from);

        List<Double> steady = new ArrayList<>();
        long start = System.nanoTime();
        while (op.opModeIsActive()) {
            double t = (System.nanoTime() - start) / 1e9;
            if (t > holdSeconds) break;
            closedLoop(to);

            double v = fw.getFilteredVelocity();
            r.peak = dir >= 0 ? Math.max(r.peak, v) : (r.peak == 0 ? v : Math.min(r.peak, v));

            if (Math.abs(to - v) <= settleBand) {
                if (Double.isNaN(r.settleTime)) r.settleTime = t;
            } else {
                r.settleTime = Double.NaN;
            }
            if (t > holdSeconds - 1.0) steady.add(v);

            status(String.format(Locale.US, "%s: target %.0f  actual %.0f in/s", name, to, v));
        }
        r.overshoot = Math.max(0, dir * (r.peak - to));
        if (!steady.isEmpty()) {
            double mean = 0;
            for (double v : steady) mean += v;
            mean /= steady.size();
            double var = 0;
            for (double v : steady) var += (v - mean) * (v - mean);
            r.steadyError = to - mean;
            r.steadyNoise = Math.sqrt(var / steady.size());
        }
        return r;
    }

    static String describe(Step r) {
        return String.format(Locale.US,
                "%s: settle %.2fs  over %.0f  err %+.1f  noise %.1f in/s",
                r.name, r.settleTime, r.overshoot, r.steadyError, r.steadyNoise);
    }

    void coast(double seconds) {
        long start = System.nanoTime();
        while (op.opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            openLoop(0);
            status("coasting");
        }
    }

    void openLoop(double power) {
        robot.sensors.update();
        fw.flywheel.setTargetPower(power);
        robot.hardwareQueue.update();
        TelemetryUtil.packet.put("FlywheelTest : power", power);
        TelemetryUtil.packet.put("FlywheelTest : velocity", fw.getRawVelocity());
        TelemetryUtil.sendTelemetry();
    }

    void closedLoop(double target) {
        robot.sensors.update();
        fw.setTargetVelocity(target);
        fw.update();
        robot.hardwareQueue.update();
        TelemetryUtil.packet.put("FlywheelTest : gain", fw.gain());
        TelemetryUtil.sendTelemetry();
    }

    private void status(String line) {
        telemetry.addLine(line);
        telemetry.addData("velocity (in/s)", "%.1f", fw.getRawVelocity());
        telemetry.update();
    }

    static double[] fitLine(List<Point> s) {
        int n = s.size();
        if (n < 2) return new double[]{Double.NaN, Double.NaN};
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (Point p : s) {
            sx += p.velocity;
            sy += p.volts;
            sxx += p.velocity * p.velocity;
            sxy += p.velocity * p.volts;
        }
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-12) return new double[]{Double.NaN, Double.NaN};
        double slope = (n * sxy - sx * sy) / den;
        return new double[]{slope, (sy - slope * sx) / n};
    }
}
