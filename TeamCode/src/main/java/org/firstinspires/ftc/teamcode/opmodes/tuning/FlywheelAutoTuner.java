package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@Autonomous(name = "Flywheel Auto Tuner", group = "test")
public class FlywheelAutoTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 530.0;

    public static double LOW_VELOCITY = 430.0;

    public static double START_ERROR = 30.0;

    public static double SWEEP_FACTOR = 0.7;

    public static double MIN_ERROR = 2.0;

    public static int STOP_AFTER_FAILURES = 2;

    public static double MAX_OVERSHOOT_FRACTION = 0.03;

    public static double MAX_NOISE = 4.0;

    public static double MAX_STEADY_ERROR = 6.0;
    public static double SETTLE_BAND = 10.0;

    public static double PREP_SECONDS = 1.5;
    public static double STEP_SECONDS = 2.5;

    private static final class Trial {
        double error, gain;
        FlywheelProcedures.Step step;
        boolean passed;
        String failure;
    }

    private Robot robot;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        FlywheelProcedures proc = new FlywheelProcedures(this, robot);

        telemetry.addLine("Flywheel Auto Tuner");
        telemetry.addLine("SAFETY: the flywheel spins to full speed. Load nothing, keep clear.");
        telemetry.addLine("Characterize -> sweep gains -> choose -> verify. About 2 minutes.");
        telemetry.addData("speeds", "%.0f -> %.0f in/s", LOW_VELOCITY, TARGET_VELOCITY);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double origKV = Flywheel.kV, origKA = Flywheel.kA, origKS = Flywheel.kS;
        double origError = Flywheel.MAX_VELOCITY_ERROR;
        boolean finished = false;
        try {
            FlywheelProcedures.Model model = proc.characterize(
                    FlywheelCharacterizer.POWER_LOW, FlywheelCharacterizer.POWER_HIGH,
                    FlywheelCharacterizer.STEPS, FlywheelCharacterizer.HOLD_SECONDS,
                    FlywheelCharacterizer.STEADY_FRACTION, FlywheelCharacterizer.REST_SECONDS,
                    FlywheelCharacterizer.KA_POWER);
            if (!opModeIsActive()) return;
            if (model.problem() != null) {
                finished = true;
                showFailure(proc, "Characterization failed: " + model.problem());
                return;
            }
            FlywheelProcedures.apply(model);

            List<Trial> trials = sweep(proc);
            if (!opModeIsActive()) return;

            List<Boolean> passed = new ArrayList<>();
            List<Double> settle = new ArrayList<>();
            double minGain = Double.MAX_VALUE, maxGain = 0;
            for (Trial t : trials) {
                passed.add(t.passed);
                settle.add(t.step.settleTime);
                minGain = Math.min(minGain, t.gain);
                maxGain = Math.max(maxGain, t.gain);
            }
            int chosen = SweepSelector.choose(passed, settle);

            boolean knobInert = maxGain > 0 && (maxGain - minGain) / maxGain < 0.05;

            if (chosen < 0) {
                Flywheel.MAX_VELOCITY_ERROR = origError;
                finished = true;
                showSweepFailure(proc, model, trials);
                return;
            }
            Flywheel.MAX_VELOCITY_ERROR = trials.get(chosen).error;

            proc.coast(1.0);
            FlywheelProcedures.Step v1 = proc.step("verify spin-up", 0, TARGET_VELOCITY,
                    STEP_SECONDS + 1.0, SETTLE_BAND);
            FlywheelProcedures.Step v2 = proc.step("verify step", TARGET_VELOCITY, LOW_VELOCITY,
                    STEP_SECONDS, SETTLE_BAND);
            if (!opModeIsActive()) return;
            finished = true;

            showResult(proc, model, trials, chosen, knobInert, v1, v2);
        } finally {
            if (!finished) {
                Flywheel.kV = origKV;
                Flywheel.kA = origKA;
                Flywheel.kS = origKS;
                Flywheel.MAX_VELOCITY_ERROR = origError;
            }
        }
    }

    private List<Trial> sweep(FlywheelProcedures proc) {
        List<Trial> trials = new ArrayList<>();
        int failuresInRow = 0;
        for (double e = START_ERROR; e >= MIN_ERROR - 1e-9 && opModeIsActive(); e *= SWEEP_FACTOR) {
            Flywheel.MAX_VELOCITY_ERROR = e;
            proc.holdClosedLoop(LOW_VELOCITY, PREP_SECONDS);

            Trial t = new Trial();
            t.error = e;
            t.gain = robot.shooter.flywheel.gain();
            t.step = proc.step(String.format(Locale.US, "trial %d (err %.1f)", trials.size() + 1, e),
                    LOW_VELOCITY, TARGET_VELOCITY, STEP_SECONDS, SETTLE_BAND);
            t.failure = judge(t.step);
            t.passed = t.failure == null;
            trials.add(t);

            failuresInRow = t.passed ? 0 : failuresInRow + 1;

            if (failuresInRow >= STOP_AFTER_FAILURES && anyPassed(trials)) break;
        }
        return trials;
    }

    private static boolean anyPassed(List<Trial> ts) {
        for (Trial t : ts) if (t.passed) return true;
        return false;
    }

    private static String judge(FlywheelProcedures.Step s) {
        if (Double.isNaN(s.settleTime)) return "never settled";
        if (s.overshoot > MAX_OVERSHOOT_FRACTION * s.to) return "overshoot";
        if (s.steadyNoise > MAX_NOISE) return "noisy";
        if (Math.abs(s.steadyError) > MAX_STEADY_ERROR) return "offset";
        return null;
    }

    private void showResult(FlywheelProcedures proc, FlywheelProcedures.Model model,
                            List<Trial> trials, int chosen, boolean knobInert,
                            FlywheelProcedures.Step v1, FlywheelProcedures.Step v2) {
        Trial c = trials.get(chosen);
        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine("=== DONE: applied live ===");
            telemetry.addData("chosen MAX_VELOCITY_ERROR", "%.2f in/s  (gain %.4f V per in/s)",
                    c.error, c.gain);
            if (knobInert) {
                telemetry.addLine("NOTE: gain changed <5% across the sweep. This wheel's response");
                telemetry.addLine("  is set by saturation/filtering, not this knob. Check kA.");
            }
            telemetry.addLine(FlywheelProcedures.describe(v1));
            telemetry.addLine(FlywheelProcedures.describe(v2));
            telemetry.addLine();
            trialTable(trials, chosen);
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(model)) telemetry.addLine(s);
            telemetry.addLine(String.format(Locale.US, "MAX_VELOCITY_ERROR = %.2f;", c.error));
            telemetry.update();
        }
    }

    private void showSweepFailure(FlywheelProcedures proc, FlywheelProcedures.Model model,
                                  List<Trial> trials) {
        String common = mostCommonFailure(trials);
        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine("=== NO SETTING PASSED ===");
            telemetry.addLine("Model was applied; MAX_VELOCITY_ERROR left unchanged.");
            telemetry.addData("most common failure", common);
            switch (common) {
                case "offset":
                    telemetry.addLine("  Feedforward is off. Check the FF_TABLE points cover");
                    telemetry.addLine("  LOW..TARGET speeds, or raise MAX_STEADY_ERROR.");
                    break;
                case "noisy":
                    telemetry.addLine("  Speed reading is noisy. Lower VELOCITY_FILTER_LOW, check");
                    telemetry.addLine("  the encoder mount, or raise MAX_NOISE if shots still land.");
                    break;
                case "overshoot":
                    telemetry.addLine("  Overshoots even when gentle: kA may be too small.");
                    break;
                default:
                    telemetry.addLine("  TARGET may be above free speed. Lower it or raise power.");
            }
            telemetry.addLine();
            trialTable(trials, -1);
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(model)) telemetry.addLine(s);
            telemetry.update();
        }
    }

    private void showFailure(FlywheelProcedures proc, String reason) {
        while (opModeIsActive()) {
            proc.openLoop(0);
            telemetry.addLine(reason);
            telemetry.addLine("Nothing was changed.");
            telemetry.update();
        }
    }

    private void trialTable(List<Trial> trials, int chosen) {
        for (int i = 0; i < trials.size(); i++) {
            Trial t = trials.get(i);
            telemetry.addLine(String.format(Locale.US, "%s err %5.1f  settle %.2fs  over %3.0f  noise %.1f  %s",
                    i == chosen ? ">" : " ", t.error, t.step.settleTime, t.step.overshoot,
                    t.step.steadyNoise, t.passed ? "ok" : t.failure));
        }
    }

    private static String mostCommonFailure(List<Trial> trials) {
        String[] kinds = {"never settled", "overshoot", "noisy", "offset"};
        String best = "never settled";
        int bestN = -1;
        for (String k : kinds) {
            int n = 0;
            for (Trial t : trials) if (k.equals(t.failure)) n++;
            if (n > bestN) {
                bestN = n;
                best = k;
            }
        }
        return best;
    }
}
