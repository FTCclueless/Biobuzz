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
import java.util.function.DoubleConsumer;

/**
 * Characterize, then tune three knobs one at a time, each with the others held:
 * 1. LATENCY             (delay the LQR predicts across)  -> keep the calmest
 * 2. MAX_VELOCITY_ERROR  (LQR stiffness), gentle -> stiff -> fastest that passes, one step back
 * 3. kI                  (integral trim), gentle -> strong -> fastest that passes, one step back
 * A stage where nothing passes keeps the value it started with.
 */
@Config
@Autonomous(name = "Flywheel Auto Tuner", group = "test")
public class FlywheelAutoTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 470.0; // keep ~1 V below battery: the FF_TABLE needs ~11.9 V at 530
    public static double LOW_VELOCITY = 370.0;
    /** Tune at no more than this share of the speed measured at POWER_HIGH, so feedback has headroom. */
    public static double REACH_FRACTION = 0.85;

    // TARGET/LOW after the reach check; a target the wheel can't hold fails every trial at ~+100.
    private double target, low;
    private String reachNote = "";

    public static double[] LATENCIES = {0.0, 0.02, 0.04, 0.06, 0.08};
    // Gentle to stiff. Below ~30 the gain stops rising (one-step deadbeat limit), so stiffer is pointless.
    public static double[] MAX_ERRORS = {250, 150, 100, 60, 30};
    public static double[] KIS = {0.03, 0.05, 0.075, 0.11, 0.17, 0.25};

    public static double MAX_OVERSHOOT_FRACTION = 0.05;
    public static double MAX_NOISE = 4.0;
    public static double MAX_STEADY_ERROR = 6.0;
    public static double SETTLE_BAND = 10.0;

    public static double PREP_SECONDS = 2.0;
    public static double STEP_SECONDS = 3.0;
    /** Runs per setting; the worst run is the one scored, so one lucky step can't pass. */
    public static int REPEATS = 2;
    /** A sweep stops after this many failures in a row once something has passed. */
    public static int STOP_AFTER_FAILURES = 2;

    private static final class Trial {
        double value;
        FlywheelProcedures.Step step;
        boolean passed;
        String failure;
    }

    private static final class Stage {
        final String name;
        final double before;
        final List<Trial> trials = new ArrayList<>();
        int chosen = -1;

        Stage(String name, double before) {
            this.name = name;
            this.before = before;
        }

        double result() { return chosen >= 0 ? trials.get(chosen).value : before; }
    }

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        FlywheelProcedures proc = new FlywheelProcedures(this, robot);

        telemetry.addLine("Flywheel Auto Tuner");
        telemetry.addLine("SAFETY: the flywheel spins to full speed. Load nothing, keep clear.");
        telemetry.addLine("Characterize -> latency -> stiffness -> kI -> verify. About 4 minutes.");
        telemetry.addData("speeds", "%.0f -> %.0f in/s", LOW_VELOCITY, TARGET_VELOCITY);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double origKV = Flywheel.kV, origKA = Flywheel.kA, origKS = Flywheel.kS;
        double origKI = Flywheel.kI, origLatency = Flywheel.LATENCY, origError = Flywheel.MAX_VELOCITY_ERROR;
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

            double top = model.points.get(model.points.size() - 1).velocity;
            target = Math.min(TARGET_VELOCITY, REACH_FRACTION * top);
            low = target * LOW_VELOCITY / TARGET_VELOCITY;
            reachNote = target < TARGET_VELOCITY
                    ? String.format(Locale.US, "TARGET lowered %.0f -> %.0f in/s: the wheel only reached %.0f in/s"
                    + " at power %.2f. If it used to reach ~550, check belts and both motor directions.",
                    TARGET_VELOCITY, target, top, FlywheelCharacterizer.POWER_HIGH)
                    : "";

            Stage latency = new Stage("LATENCY", Flywheel.LATENCY);
            run(proc, latency, LATENCIES, v -> Flywheel.LATENCY = v, false);
            if (!opModeIsActive()) return;
            latency.chosen = calmest(latency.trials);
            Flywheel.LATENCY = latency.result();

            Stage stiffness = new Stage("MAX_VELOCITY_ERROR", Flywheel.MAX_VELOCITY_ERROR);
            run(proc, stiffness, MAX_ERRORS, v -> Flywheel.MAX_VELOCITY_ERROR = v, true);
            if (!opModeIsActive()) return;
            stiffness.chosen = choose(stiffness.trials);
            Flywheel.MAX_VELOCITY_ERROR = stiffness.result();

            Stage integral = new Stage("kI", Flywheel.kI);
            run(proc, integral, KIS, v -> Flywheel.kI = v, true);
            if (!opModeIsActive()) return;
            integral.chosen = choose(integral.trials);
            Flywheel.kI = integral.result();

            proc.coast(1.0);
            FlywheelProcedures.Step v1 = proc.step("verify spin-up", 0, target,
                    STEP_SECONDS + 1.0, SETTLE_BAND);
            FlywheelProcedures.Step v2 = proc.step("verify step down", target, low,
                    STEP_SECONDS, SETTLE_BAND);
            if (!opModeIsActive()) return;
            finished = true;

            showResult(proc, model, new Stage[]{latency, stiffness, integral}, v1, v2);
        } finally {
            if (!finished) {
                Flywheel.kV = origKV;
                Flywheel.kA = origKA;
                Flywheel.kS = origKS;
                Flywheel.kI = origKI;
                Flywheel.LATENCY = origLatency;
                Flywheel.MAX_VELOCITY_ERROR = origError;
            }
        }
    }

    /** Tries each value in order; a monotone sweep stops early once it has passed and then failed twice. */
    private void run(FlywheelProcedures proc, Stage stage, double[] values, DoubleConsumer set,
                     boolean monotone) {
        int failuresInRow = 0;
        for (double v : values) {
            if (!opModeIsActive()) break;
            set.accept(v);
            Trial t = trial(proc, String.format(Locale.US, "%s %.3f", stage.name, v));
            t.value = v;
            stage.trials.add(t);
            failuresInRow = t.passed ? 0 : failuresInRow + 1;
            if (monotone && failuresInRow >= STOP_AFTER_FAILURES && anyPassed(stage.trials)) break;
        }
        set.accept(stage.before); // restored until the caller applies the chosen value
    }

    private Trial trial(FlywheelProcedures proc, String name) {
        Trial t = new Trial();
        for (int r = 0; r < Math.max(1, REPEATS) && opModeIsActive(); r++) {
            proc.holdClosedLoop(low, PREP_SECONDS);
            FlywheelProcedures.Step s = proc.step(name + " #" + (r + 1),
                    low, target, STEP_SECONDS, SETTLE_BAND);
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

    private static int choose(List<Trial> ts) {
        List<Boolean> passed = new ArrayList<>();
        List<Double> settle = new ArrayList<>();
        for (Trial t : ts) {
            passed.add(t.passed);
            settle.add(t.step.settleTime);
        }
        return SweepSelector.choose(passed, settle);
    }

    private static int calmest(List<Trial> ts) {
        int best = -1;
        for (int i = 0; i < ts.size(); i++) {
            if (best < 0 || calmness(ts.get(i).step) < calmness(ts.get(best).step)) best = i;
        }
        return best;
    }

    private static double calmness(FlywheelProcedures.Step s) {
        double noise = Double.isNaN(s.steadyNoise) ? 1e6 : s.steadyNoise;
        double err = Double.isNaN(s.steadyError) ? 1e6 : Math.abs(s.steadyError);
        return s.overshoot + 3 * noise + err;
    }

    private static boolean anyPassed(List<Trial> ts) {
        for (Trial t : ts) if (t.passed) return true;
        return false;
    }

    private static String judge(FlywheelProcedures.Step s) {
        // First: at full output no gain can help, so blaming overshoot/offset would mislead.
        if (s.saturatedFraction > 0.5) return "maxed out";
        if (Double.isNaN(s.settleTime)) return "never settled";
        if (s.overshoot > MAX_OVERSHOOT_FRACTION * s.to) return "overshoot";
        if (s.steadyNoise > MAX_NOISE) return "noisy";
        if (Math.abs(s.steadyError) > MAX_STEADY_ERROR) return "offset";
        return null;
    }

    private void showResult(FlywheelProcedures proc, FlywheelProcedures.Model model, Stage[] stages,
                            FlywheelProcedures.Step v1, FlywheelProcedures.Step v2) {
        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine("=== DONE: applied live ===");
            telemetry.addData("tuned at", "%.0f -> %.0f in/s", low, target);
            if (!reachNote.isEmpty()) telemetry.addLine(reachNote);
            telemetry.addLine(FlywheelProcedures.describe(v1));
            telemetry.addLine(FlywheelProcedures.describe(v2));
            for (Stage st : stages) {
                telemetry.addLine();
                telemetry.addLine(st.chosen >= 0
                        ? String.format(Locale.US, "--- %s: chose %.3f", st.name, st.result())
                        : String.format(Locale.US, "--- %s: nothing passed (%s), kept %.3f",
                                st.name, mostCommonFailure(st.trials), st.before));
                for (int i = 0; i < st.trials.size(); i++) {
                    Trial t = st.trials.get(i);
                    telemetry.addLine(String.format(Locale.US,
                            "%s %7.3f  settle %.2fs  over %3.0f  err %+.1f  noise %.1f  maxed %.0f%%  %s",
                            i == st.chosen ? ">" : " ", t.value, t.step.settleTime, t.step.overshoot,
                            t.step.steadyError, t.step.steadyNoise, 100 * t.step.saturatedFraction,
                            t.passed ? "ok" : t.failure));
                }
            }
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(model)) telemetry.addLine(s);
            telemetry.addLine(String.format(Locale.US, "LATENCY = %.2f;", stages[0].result()));
            telemetry.addLine(String.format(Locale.US, "MAX_VELOCITY_ERROR = %.1f;", stages[1].result()));
            telemetry.addLine(String.format(Locale.US, "kI = %.3f;", stages[2].result()));
            if (stages[1].chosen < 0 || stages[2].chosen < 0) {
                telemetry.addLine();
                telemetry.addLine("Some stage found nothing that passed. Most likely (err + = slower than target):");
                telemetry.addLine("  maxed out     -> full power can't hold the target: lower REACH_FRACTION,"
                        + " or the wheel is weak (belts, a motor reversed or unplugged)");
                telemetry.addLine("  never settled -> TARGET near free speed: lower TARGET_VELOCITY");
                telemetry.addLine("  overshoot     -> add gentler MAX_ERRORS (e.g. 400) or lower KIS");
                telemetry.addLine("  noisy         -> encoder: check cable/mount, or raise MAX_NOISE");
                telemetry.addLine("  offset        -> raise the larger KIS values or STEP_SECONDS");
            }
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

    private static String mostCommonFailure(List<Trial> trials) {
        String[] kinds = {"maxed out", "never settled", "overshoot", "noisy", "offset"};
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
