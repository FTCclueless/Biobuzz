package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures the three flywheel model constants that Flywheel's LQR needs.
 *
 * How it works:
 *   kV and kS come from holding several power levels until the speed stops changing. Plot volts
 *   against steady speed and you get a straight line: the slope is kV (volts per in/s) and the
 *   intercept is kS (volts lost to friction before the wheel moves at all).
 *
 *   kA comes from how long the wheel takes to reach 63% of its final speed from rest. That time
 *   is the time constant tau, and kA = kV * tau.
 *
 * Safety: the wheel spins to full speed. Clear the shooter, make sure nothing can fall into it,
 * and keep hands away. Nothing should be loaded.
 */
@Config
@Autonomous(name = "Flywheel Characterizer", group = "test")
public class FlywheelCharacterizer extends LinearOpMode {
    /** Power levels to sample. Below about 0.3 the wheel may not overcome friction cleanly. */
    public static double POWER_LOW = 0.35;
    public static double POWER_HIGH = 0.95;
    public static int STEPS = 5;
    /** Seconds to hold each power level. Must be long enough for the speed to flatten. */
    public static double HOLD_SECONDS = 2.5;
    /** Fraction at the end of each hold that counts as steady state. */
    public static double STEADY_FRACTION = 0.3;
    /** Seconds to coast between levels. */
    public static double REST_SECONDS = 1.5;
    /** Power used for the spin-up run that measures kA. */
    public static double KA_POWER = 0.8;

    private Robot robot;

    private static final class Sample {
        double power;
        double volts;
        double velocity;
    }

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        telemetry.addLine("Flywheel Characterizer");
        telemetry.addLine();
        telemetry.addLine("SAFETY: the flywheel spins to full speed.");
        telemetry.addLine("Clear the shooter, load nothing, keep clear.");
        telemetry.addLine();
        telemetry.addData("steps", "%d levels from %.2f to %.2f", STEPS, POWER_LOW, POWER_HIGH);
        telemetry.addData("estimated time (s)", "%.0f",
                STEPS * (HOLD_SECONDS + REST_SECONDS) + HOLD_SECONDS + REST_SECONDS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        List<Sample> samples = new ArrayList<>();
        for (int i = 0; i < STEPS && opModeIsActive(); i++) {
            double p = STEPS == 1 ? POWER_HIGH
                    : POWER_LOW + (POWER_HIGH - POWER_LOW) * i / (STEPS - 1.0);
            coast(REST_SECONDS);
            Sample s = holdAndMeasure(p);
            if (s != null) samples.add(s);
        }

        coast(REST_SECONDS);
        double tau = measureTimeConstant();

        double[] fit = leastSquares(samples);
        double kV = fit[0];
        double kS = fit[1];
        double kA = kV * tau;

        stopMotor();
        report(samples, kV, kS, kA, tau);
    }

    /** Holds one power level and averages the velocity over the last part of the hold. */
    private Sample holdAndMeasure(double power) {
        long start = System.nanoTime();
        double sumV = 0, sumVolts = 0;
        int n = 0;

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            if (elapsed > HOLD_SECONDS) break;

            tick(power);

            if (elapsed > HOLD_SECONDS * (1.0 - STEADY_FRACTION)) {
                sumV += robot.sensors.getFlywheelVelocity();
                sumVolts += power * robot.sensors.getVoltage();
                n++;
            }

            telemetry.addData("holding power", "%.2f", power);
            telemetry.addData("velocity (in/s)", "%.1f", robot.sensors.getFlywheelVelocity());
            telemetry.addData("elapsed", "%.1f / %.1f", elapsed, HOLD_SECONDS);
            telemetry.update();
        }

        if (n == 0) return null;
        Sample s = new Sample();
        s.power = power;
        s.velocity = sumV / n;
        s.volts = sumVolts / n;
        return s;
    }

    /**
     * Spins up from rest and returns the time to reach 63.2% of the final speed. The final speed
     * is taken from the end of the run, so the wheel must actually settle within HOLD_SECONDS.
     */
    private double measureTimeConstant() {
        List<Double> t = new ArrayList<>(), v = new ArrayList<>();
        long start = System.nanoTime();

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            if (elapsed > HOLD_SECONDS) break;

            tick(KA_POWER);

            t.add(elapsed);
            v.add(robot.sensors.getFlywheelVelocity());

            TelemetryUtil.packet.put("Characterizer : spin-up velocity",
                    robot.sensors.getFlywheelVelocity());
            telemetry.addLine("measuring spin-up (kA)");
            telemetry.addData("velocity (in/s)", "%.1f", robot.sensors.getFlywheelVelocity());
            telemetry.update();
        }

        if (t.size() < 8) return Double.NaN;

        // Final speed: average the last 20% of the run.
        int from = (int) (t.size() * 0.8);
        double finalV = 0;
        for (int i = from; i < t.size(); i++) finalV += v.get(i);
        finalV /= (t.size() - from);
        if (finalV < 1e-6) return Double.NaN;

        double target = 0.632 * finalV;
        for (int i = 0; i < t.size(); i++) {
            if (v.get(i) >= target) return t.get(i);
        }
        return Double.NaN;
    }

    /** Fits volts = kV * velocity + kS. Returns {kV, kS}. */
    private static double[] leastSquares(List<Sample> s) {
        int n = s.size();
        if (n < 2) return new double[]{Double.NaN, Double.NaN};
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (Sample p : s) {
            sx += p.velocity;
            sy += p.volts;
            sxx += p.velocity * p.velocity;
            sxy += p.velocity * p.volts;
        }
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-12) return new double[]{Double.NaN, Double.NaN};
        double slope = (n * sxy - sx * sy) / den;
        double intercept = (sy - slope * sx) / n;
        return new double[]{slope, intercept};
    }

    private void coast(double seconds) {
        long start = System.nanoTime();
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            tick(0);
            telemetry.addLine("coasting");
            telemetry.addData("velocity (in/s)", "%.1f", robot.sensors.getFlywheelVelocity());
            telemetry.update();
        }
    }

    private void stopMotor() {
        tick(0);
    }

    /**
     * One control loop that talks to the flywheel directly.
     *
     * Deliberately NOT robot.update(): Shooter.update() commands the flywheel every loop even
     * when idle, so it would immediately overwrite the open-loop power this test depends on.
     * Sensors and the hardware queue still run, so velocity and voltage stay live.
     */
    private void tick(double power) {
        robot.sensors.update();
        robot.shooter.flywheel.flywheel.setTargetPower(power);
        robot.hardwareQueue.update();
        TelemetryUtil.packet.put("Characterizer : power", power);
        TelemetryUtil.packet.put("Characterizer : velocity", robot.sensors.getFlywheelVelocity());
        TelemetryUtil.sendTelemetry();
    }

    private void report(List<Sample> samples, double kV, double kS, double kA, double tau) {
        while (opModeIsActive()) {
            tick(0);

            telemetry.addLine("=== measured points ===");
            for (Sample s : samples) {
                telemetry.addData(String.format(Locale.US, "p=%.2f", s.power),
                        String.format(Locale.US, "%.1f in/s at %.2f V", s.velocity, s.volts));
            }
            telemetry.addLine();
            telemetry.addData("time constant tau (s)", "%.3f", tau);
            telemetry.addLine();
            telemetry.addLine("=== paste into Flywheel FF_TABLE ===");
            for (Sample s : samples) {
                telemetry.addLine(String.format(Locale.US,
                        "FF_TABLE.put(%.1f, %.3f);", s.velocity, s.volts));
            }
            telemetry.addLine();
            telemetry.addLine("=== paste into Flywheel ===");
            telemetry.addLine(String.format(Locale.US, "kV = %.6f;", kV));
            telemetry.addLine(String.format(Locale.US, "kA = %.6f;", kA));
            telemetry.addLine(String.format(Locale.US, "kS = %.4f;", kS));
            telemetry.addLine();

            if (Double.isNaN(kV) || kV <= 0) {
                telemetry.addLine("BAD kV: need at least 2 good points with rising speed.");
            } else if (Double.isNaN(tau)) {
                telemetry.addLine("BAD tau: the wheel never settled. Raise HOLD_SECONDS.");
            } else if (kS < 0) {
                telemetry.addLine("kS came out negative, which is not physical. Usually means");
                telemetry.addLine("  the lowest power levels never reached steady state --");
                telemetry.addLine("  raise POWER_LOW or HOLD_SECONDS and re-run.");
            } else {
                telemetry.addLine("Looks sane. Enter these in Flywheel, then run Flywheel Tuner.");
            }
            telemetry.update();
        }
    }
}
