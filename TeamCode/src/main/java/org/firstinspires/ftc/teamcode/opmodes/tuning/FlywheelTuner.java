package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

import java.util.Locale;

@Config
@Autonomous(name = "Flywheel Tuner", group = "test")
public class FlywheelTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 530.0;

    public static double SECOND_VELOCITY = 430.0;
    public static double HOLD_SECONDS = 4.0;
    public static double SETTLE_BAND = 10.0;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        FlywheelProcedures proc = new FlywheelProcedures(this, robot);

        telemetry.addLine("Flywheel Tuner");
        telemetry.addLine("SAFETY: the flywheel spins to full speed. Load nothing.");
        telemetry.addData("model", "kV=%.6f kA=%.6f kS=%.4f",
                Flywheel.kV, Flywheel.kA, Flywheel.kS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        FlywheelProcedures.Step up = proc.step("spin-up", 0, TARGET_VELOCITY,
                HOLD_SECONDS, SETTLE_BAND);
        FlywheelProcedures.Step down = SECOND_VELOCITY != TARGET_VELOCITY
                ? proc.step("step", TARGET_VELOCITY, SECOND_VELOCITY, HOLD_SECONDS, SETTLE_BAND)
                : null;

        while (opModeIsActive()) {
            proc.closedLoop(0);
            telemetry.addLine(FlywheelProcedures.describe(up));
            if (down != null) telemetry.addLine(FlywheelProcedures.describe(down));
            telemetry.addLine();
            telemetry.addLine("=== next change ===");
            for (String s : advice(up)) telemetry.addLine(s);
            telemetry.update();
        }
    }

    private static String[] advice(FlywheelProcedures.Step r) {
        if (Double.isNaN(r.settleTime)) {
            return new String[]{"NEVER SETTLED within the hold.",
                    "  -> target may be above free speed, or kV is wrong. Re-characterize."};
        }
        if (Math.abs(r.steadyError) > 5 && Math.abs(r.steadyError) > 3 * Math.max(1, r.overshoot)) {
            return new String[]{
                    String.format(Locale.US, "Sits %+.1f in/s off target.", r.steadyError),
                    "  Feedforward error, not a gain problem: re-check kV/kS or FF_TABLE."};
        }
        if (r.overshoot > 0.05 * r.to) {
            return new String[]{String.format(Locale.US, "Overshoots by %.0f in/s.", r.overshoot),
                    "  -> raise MAX_VELOCITY_ERROR, or lower MAX_EFFORT."};
        }
        return new String[]{
                String.format(Locale.US, "Good: settles in %.2fs, noise %.1f in/s.",
                        r.settleTime, r.steadyNoise),
                "  Or run Flywheel Auto Tuner to search for the fastest stable setting."};
    }
}
