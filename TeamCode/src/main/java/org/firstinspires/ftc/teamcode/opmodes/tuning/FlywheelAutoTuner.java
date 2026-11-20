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
    public static double TARGET_VELOCITY = 470.0; // keep ~1 V below battery: the FF_TABLE needs ~11.9 V at 530

    public static double LOW_VELOCITY = 370.0;

    // Sweeps Flywheel.kI, gentle to aggressive. MAX_VELOCITY_ERROR is not swept: at this
    // loop rate its LQR gain barely changes (it saturates near the one-step deadbeat gain).
    public static double START_KI = 0.05;

    public static double SWEEP_FACTOR = 1.5;

    public static double MAX_KI = 0.8;

    /** Stage 1 tries each latency at the current kI and keeps the calmest (least overshoot + noise). */
    public static double[] LATENCIES = {0.0, 0.02, 0.04, 0.06, 0.08};

    public static int STOP_AFTER_FAILURES = 2;

    public static double MAX_OVERSHOOT_FRACTION = 0.05;

    public static double MAX_NOISE = 4.0;

    public static double MAX_STEADY_ERROR = 6.0;
    public static double SETTLE_BAND = 10.0;

    public static double PREP_SECONDS = 1.5;
    public static double STEP_SECONDS = 3.0;
    /** Runs per setting; the worst run is the one scored, so one lucky step can't pass. */
    public static int REPEATS = 2;

    private static final class Trial {
        double kI, latency;
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
        telemetry.addLine("Characterize -> find latency -> sweep kI -> verify. About 4 minutes.");
        telemetry.addData("speeds", "%.0f -> %.0f in/s", LOW_VELOCITY, TARGET_VELOCITY);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double origKV = Flywheel.kV, origKA = Flywheel.kA, origKS = Flywheel.kS;
        double origKI = Flywheel.kI, origLatency = Flywheel.LATENCY;
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

            List<Trial> latencyTrials = sweepLatency(proc);
            if (!opModeIsActive()) return;
            Flywheel.LATENCY = calmest(latencyTrials).latency;

            List<Trial> trials = sweep(proc);
            if (!opModeIsActive()) return;

            List<Boolean> passed = new ArrayList<>();
            List<Double> settle = new ArrayList<>();
            for (Trial t : trials) {
                passed.add(t.passed);
                settle.add(t.step.settleTime);
            }
            int chosen = SweepSelector.choose(passed, settle);

            if (chosen < 0) {
                Flywheel.kI = origKI;
                finished = true;
                showSweepFailure(proc, model, latencyTrials, trials);
                return;
            }
            Flywheel.kI = trials.get(chosen).kI;

            proc.coast(1.0);
            FlywheelProcedures.Step v1 = proc.step("verify spin-up", 0, TARGET_VELOCITY,
                    STEP_SECONDS + 1.0, SETTLE_BAND);
            FlywheelProcedures.Step v2 = proc.step("verify step", TARGET_VELOCITY, LOW_VELOCITY,
                    STEP_SECONDS, SETTLE_BAND);
            if (!opModeIsActive()) return;
            finished = true;

            showResult(proc, model, latencyTrials, trials, chosen, v1, v2);
        } finally {
            if (!finished) {
                Flywheel.kV = origKV;
                Flywheel.kA = origKA;
                Flywheel.kS = origKS;
                Flywheel.kI = origKI;
                Flywheel.LATENCY = origLatency;
            }
        }
    }

    private List<Trial> sweepLatency(FlywheelProcedures proc) {
        List<Trial> trials = new ArrayList<>();
        for (double l : LATENCIES) {
            if (!opModeIsActive()) break;
            Flywheel.LATENCY = l;
            trials.add(trial(proc, String.format(Locale.US, "latency %.2fs", l)));
        }
        return trials;
    }

    private static Trial calmest(List<Trial> ts) {
        Trial best = ts.get(0);
        for (Trial t : ts) if (calmness(t) < calmness(best)) best = t;
        return best;
    }

    private static double calmness(Trial t) { return calmness(t.step); }

    private static double calmness(FlywheelProcedures.Step s) {
        double noise = Double.isNaN(s.steadyNoise) ? 1e6 : s.steadyNoise;
        double err = Double.isNaN(s.steadyError) ? 1e6 : Math.abs(s.steadyError);
        return s.overshoot + 3 * noise + err;
    }

    private Trial trial(FlywheelProcedures proc, String name) {
        Trial t = new Trial();
        t.kI = Flywheel.kI;
        t.latency = Flywheel.LATENCY;
        for (int r = 0; r < Math.max(1, REPEATS) && opModeIsActive(); r++) {
            proc.holdClosedLoop(LOW_VELOCITY, PREP_SECONDS);
            FlywheelProcedures.Step s = proc.step(name + " #" + (r + 1),
                    LOW_VELOCITY, TARGET_VELOCITY, STEP_SECONDS, SETTLE_BAND);
            String f = judge(s);
            // Keep the worst run: any failure beats a pass, then the less calm of the two.
            if (t.step == null || (f != null && t.failure == null)
                    || ((f == null) == (t.failure == null) && calmness(s) > calmness(t.step))) {
                t.step = s;
                t.failure = f;
            }
        }
        t.passed = t.failure == null;
        return t;
    }

    private List<Trial> sweep(FlywheelProcedures proc) {
        List<Trial> trials = new ArrayList<>();
        int failuresInRow = 0;
        for (double k = START_KI; k <= MAX_KI + 1e-9 && opModeIsActive(); k *= SWEEP_FACTOR) {
            Flywheel.kI = k;
            Trial t = trial(proc, String.format(Locale.US, "trial %d (kI %.3f)", trials.size() + 1, k));
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
                            List<Trial> latencyTrials, List<Trial> trials, int chosen,
                            FlywheelProcedures.Step v1, FlywheelProcedures.Step v2) {
        Trial c = trials.get(chosen);
        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine("=== DONE: applied live ===");
            telemetry.addData("chosen kI", "%.3f V per (in/s*s)", c.kI);
            telemetry.addData("chosen LATENCY", "%.2f s", c.latency);
            telemetry.addLine(FlywheelProcedures.describe(v1));
            telemetry.addLine(FlywheelProcedures.describe(v2));
            telemetry.addLine();
            trialTable(latencyTrials, -1);
            trialTable(trials, chosen);
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(model)) telemetry.addLine(s);
            telemetry.addLine(String.format(Locale.US, "kI = %.3f;", c.kI));
            telemetry.addLine(String.format(Locale.US, "LATENCY = %.2f;", c.latency));
            telemetry.update();
        }
    }

    private void showSweepFailure(FlywheelProcedures proc, FlywheelProcedures.Model model,
                                  List<Trial> latencyTrials, List<Trial> trials) {
        String common = mostCommonFailure(trials);
        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine("=== NO SETTING PASSED ===");
            telemetry.addLine("Model was applied; kI left unchanged.");
            telemetry.addData("most common failure", common);
            switch (common) {
                case "offset":
                    telemetry.addLine("  Integral too weak to remove the feedforward error in time:");
                    telemetry.addLine("  raise MAX_KI or STEP_SECONDS, or re-characterize warm.");
                    break;
                case "noisy":
                    telemetry.addLine("  Speed reading is noisy. Lower VELOCITY_FILTER_LOW, check");
                    telemetry.addLine("  the encoder mount, or raise MAX_NOISE if shots still land.");
                    break;
                case "overshoot":
                    telemetry.addLine("  Overshoots even at the lowest kI: lower START_KI, or kA is too big.");
                    break;
                default:
                    telemetry.addLine("  TARGET may be above free speed. Lower it or raise power.");
            }
            telemetry.addLine();
            trialTable(latencyTrials, -1);
            trialTable(trials, -1);
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(model)) telemetry.addLine(s);
            telemetry.addLine(String.format(Locale.US, "LATENCY = %.2f;  (calmest, applied)", Flywheel.LATENCY));
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
            telemetry.addLine(String.format(Locale.US, "%s kI %.3f lat %.2f  settle %.2fs  over %3.0f  err %+.1f  noise %.1f  %s",
                    i == chosen ? ">" : " ", t.kI, t.latency, t.step.settleTime, t.step.overshoot,
                    t.step.steadyError, t.step.steadyNoise, t.passed ? "ok" : t.failure));
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
