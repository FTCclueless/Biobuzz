package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

import java.util.Locale;

@Config
@Autonomous(name = "Flywheel Characterizer", group = "test")
public class FlywheelCharacterizer extends LinearOpMode {
    public static double POWER_LOW = 0.35;
    public static double POWER_HIGH = 0.95;
    public static int STEPS = 5;
    public static double HOLD_SECONDS = 2.5;
    public static double STEADY_FRACTION = 0.3;
    public static double REST_SECONDS = 1.5;
    public static double KA_POWER = 0.8;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        FlywheelProcedures proc = new FlywheelProcedures(this, robot);

        telemetry.addLine("Flywheel Characterizer");
        telemetry.addLine("SAFETY: the flywheel spins to full speed. Load nothing, keep clear.");
        telemetry.addData("estimated time (s)", "%.0f",
                (STEPS + 1) * (HOLD_SECONDS + REST_SECONDS));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        FlywheelProcedures.Model m = proc.characterize(POWER_LOW, POWER_HIGH, STEPS,
                HOLD_SECONDS, STEADY_FRACTION, REST_SECONDS, KA_POWER);

        while (opModeIsActive()) {
            proc.openLoop(0);
            for (FlywheelProcedures.Point p : m.points) {
                telemetry.addData(String.format(Locale.US, "p=%.2f", p.power),
                        String.format(Locale.US, "%.1f in/s at %.2f V", p.velocity, p.volts));
            }
            telemetry.addData("tau (s)", "%.3f", m.tau);
            telemetry.addLine();
            for (String s : FlywheelProcedures.pasteLines(m)) telemetry.addLine(s);
            telemetry.addLine();
            String problem = m.problem();
            telemetry.addLine(problem == null
                    ? "Looks sane. Paste these into Flywheel, then run Flywheel Tuner."
                    : "PROBLEM: " + problem);
            telemetry.update();
        }
    }
}
