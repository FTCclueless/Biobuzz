package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Locale;

/**
 * Step-tests the flywheel LQR and reports how well it tracked.
 *
 * Run this AFTER Flywheel Characterizer. It drives the closed loop between two speeds and
 * measures spin-up time, overshoot, and steady-state error -- the three things that decide
 * whether a shot goes in. Graph "Flywheel : Target Velocity" against "Flywheel : Filtered
 * Velocity" in Dashboard while it runs.
 *
 * Safety: the wheel spins to full speed. Clear the shooter, load nothing, keep clear.
 */
@Config
@Autonomous(name = "Flywheel Tuner", group = "test")
public class FlywheelTuner extends LinearOpMode {
    /** Speed to hold, in/s. Use a real shooting speed from ShotTable2 (last season: 430-640). */
    public static double TARGET_VELOCITY = 530.0;
    /** Second speed for the step-down test. Set equal to TARGET_VELOCITY to skip. */
    public static double SECOND_VELOCITY = 430.0;
    /** Seconds to hold each speed. */
    public static double HOLD_SECONDS = 4.0;
    /** Error, in/s, that counts as settled. */
    public static double SETTLE_BAND = 10.0;

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        telemetry.addLine("Flywheel Tuner");
        telemetry.addLine();
        telemetry.addLine("SAFETY: the flywheel spins to full speed. Load nothing.");
        telemetry.addLine();
        telemetry.addData("model", "kV=%.6f kA=%.6f kS=%.4f",
                Flywheel.kV, Flywheel.kA, Flywheel.kS);
        telemetry.addData("step", "0 -> %.0f -> %.0f in/s", TARGET_VELOCITY, SECOND_VELOCITY);
        telemetry.addLine();
        telemetry.addLine("Run Flywheel Characterizer first if the model is untouched defaults.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Result up = step(TARGET_VELOCITY, "spin-up from rest");
        Result down = SECOND_VELOCITY != TARGET_VELOCITY
                ? step(SECOND_VELOCITY, "step to second speed") : null;

        tick(0);

        report(up, down);
    }

    private static final class Result {
        String name;
        double target;
        double settleTime = Double.NaN;
        double overshoot = 0;
        double steadyError = 0;
        double peak = 0;
    }

    private Result step(double target, String name) {
        Result r = new Result();
        r.name = name;
        r.target = target;

        long start = System.nanoTime();
        double sumErr = 0;
        int nSteady = 0;
        boolean settled = false;

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            if (elapsed > HOLD_SECONDS) break;

            tick(target);

            double v = robot.shooter.flywheel.getFilteredVelocity();
            r.peak = Math.max(r.peak, v);

            if (!settled && Math.abs(target - v) <= SETTLE_BAND) {
                settled = true;
                r.settleTime = elapsed;
            }
            // Average the error over the last second, once the transient is done.
            if (elapsed > HOLD_SECONDS - 1.0) {
                sumErr += target - v;
                nSteady++;
            }

            telemetry.addData("phase", name);
            telemetry.addData("target / actual", "%.0f / %.0f in/s", target, v);
            telemetry.addData("elapsed", "%.1f / %.1f", elapsed, HOLD_SECONDS);
            telemetry.update();
        }

        r.overshoot = Math.max(0, r.peak - target);
        r.steadyError = nSteady > 0 ? sumErr / nSteady : Double.NaN;
        return r;
    }

    /**
     * One control loop running Flywheel's own update(), which is the thing under test.
     *
     * Deliberately NOT robot.update(): Shooter.update() commands a flywheel speed of its own
     * every loop, which would overwrite the step this test is trying to measure.
     */
    private void tick(double target) {
        robot.sensors.update();
        robot.shooter.flywheel.setTargetVelocity(target);
        robot.shooter.flywheel.update();
        robot.hardwareQueue.update();
        TelemetryUtil.packet.put("Tuner : gain", robot.shooter.flywheel.gain());
        TelemetryUtil.sendTelemetry();
    }

    private void report(Result up, Result down) {
        while (opModeIsActive()) {
            tick(0);

            line(up);
            if (down != null) line(down);
            telemetry.addLine();
            telemetry.addLine("=== next change ===");
            for (String s : advice(up)) telemetry.addLine(s);
            telemetry.update();
        }
    }

    private void line(Result r) {
        telemetry.addLine(String.format(Locale.US,
                "%s: settle %.2fs, overshoot %.0f, steady err %+.1f in/s",
                r.name, r.settleTime, r.overshoot, r.steadyError));
    }

    private static String[] advice(Result r) {
        if (Double.isNaN(r.settleTime)) {
            return new String[]{
                    "NEVER REACHED TARGET within the hold.",
                    "  -> the target may be above free speed, or kV is too small.",
                    "  Re-run Flywheel Characterizer and check the top measured speed."};
        }
        if (Math.abs(r.steadyError) > 3 * Math.max(1, r.overshoot) && Math.abs(r.steadyError) > 5) {
            return new String[]{
                    String.format(Locale.US,
                            "Sits %+.1f in/s off target and stays there.", r.steadyError),
                    "  This is a feedforward error, not a gain problem.",
                    "  -> re-check kV and kS from the Characterizer.",
                    "  Only if those are right: lower MAX_VELOCITY_ERROR to stiffen feedback."};
        }
        if (r.overshoot > 0.05 * r.target) {
            return new String[]{
                    String.format(Locale.US, "Overshoots by %.0f in/s.", r.overshoot),
                    "  -> raise MAX_VELOCITY_ERROR (accept more error, act less hard),",
                    "     or lower MAX_EFFORT. Check kA is not too small."};
        }
        return new String[]{
                String.format(Locale.US,
                        "Good: settles in %.2fs with %+.1f in/s left over.",
                        r.settleTime, r.steadyError),
                "  For faster recovery between shots, lower MAX_VELOCITY_ERROR a little",
                "  and re-run until it starts to overshoot, then back off."};
    }
}
