package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.SlipDetector;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.Locale;

@Config
@Autonomous(name = "Slip Test", group = "test")
public class SlipTest extends LinearOpMode {
    public static boolean STRAFE = false;

    public static double TICKS_PER_REV = 537.7;
    public static double WHEEL_DIAMETER = 3.78;
    public static boolean INVERT_ENCODER = false;

    public static double RAMP_SECONDS = 2.0;
    public static double MAX_POWER = 1.0;
    public static double MAX_TRAVEL = 90.0;

    public static double SLIP_RATIO = 1.18;
    public static double MIN_SPEED = 3.0;

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        MergeLocalizer.useCamera = false;

        SlipDetector slip = new SlipDetector(SLIP_RATIO, MIN_SPEED, 0.25);
        double inchesPerTick = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        telemetry.addLine("Slip Test");
        telemetry.addLine();
        telemetry.addLine("Requires the rightRear DRIVE encoder in the rightRear port.");
        telemetry.addLine("See the class comment before running.");
        telemetry.addData("axis", STRAFE ? "STRAFE -> A_STRAFE" : "FORWARD -> A_FORWARD");
        telemetry.addData("clear run needed (in)", "%.0f", MAX_TRAVEL);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();

        double startX = Globals.ROBOT_POSITION.x, startY = Globals.ROBOT_POSITION.y;
        long start = System.nanoTime();
        boolean everSlipped = false;
        double peakRatio = 0, powerAtSlip = Double.NaN, travel = 0;

        double baselineRatio = 1.0;
        int baselineN = 0;
        double baselineSum = 0;

        while (opModeIsActive()) {
            double elapsed = (System.nanoTime() - start) / 1e9;
            double power = Math.min(MAX_POWER, MAX_POWER * elapsed / RAMP_SECONDS);

            if (STRAFE) robot.drivetrain.setMotorPowers(-power, power, -power, power);
            else        robot.drivetrain.setMotorPowers(power, power, power, power);
            robot.hardwareQueue.update();

            robot.update();

            double ticksPerSec = robot.drivetrain.rightRear.getVelocity();
            double wheelSpeed = ticksPerSec * inchesPerTick * (INVERT_ENCODER ? -1 : 1);
            Vector2 wheelVel = STRAFE ? new Vector2(0, wheelSpeed) : new Vector2(wheelSpeed, 0);

            Vector2 odoVel = new Vector2(ROBOT_VELOCITY.x, ROBOT_VELOCITY.y);

            slip.update(wheelVel, odoVel, robot.sensors.loopTime);

            if (power < 0.30 * MAX_POWER && odoVel.mag() > MIN_SPEED) {
                baselineSum += slip.slipRatio();
                baselineN++;
                baselineRatio = baselineSum / baselineN;
            }

            if (slip.isSlipping() && !everSlipped) {
                everSlipped = true;
                powerAtSlip = power;
            }
            peakRatio = Math.max(peakRatio, slip.slipRatio());

            travel = Math.hypot(Globals.ROBOT_POSITION.x - startX,
                                Globals.ROBOT_POSITION.y - startY);

            TelemetryUtil.packet.put("Slip : power", power);
            TelemetryUtil.packet.put("Slip : wheel speed", wheelSpeed);
            TelemetryUtil.packet.put("Slip : ground speed", odoVel.mag());
            TelemetryUtil.packet.put("Slip : ratio", slip.slipRatio());
            TelemetryUtil.packet.put("Slip : peak traction accel", slip.peakTractionAccel());

            telemetry.addData("power", "%.2f", power);
            telemetry.addData("wheel / ground (in/s)", "%.1f / %.1f", wheelSpeed, odoVel.mag());
            telemetry.addData("ratio", "%.2f%s", slip.slipRatio(), slip.isSlipping() ? "  SLIPPING" : "");
            telemetry.addData("traction accel so far", "%.0f", slip.peakTractionAccel());
            telemetry.addData("travelled", "%.0f / %.0f", travel, MAX_TRAVEL);
            telemetry.update();

            if (travel > MAX_TRAVEL) break;
            if (elapsed > RAMP_SECONDS + 1.5) break;
        }

        robot.drivetrain.stopAllMotors();
        robot.hardwareQueue.update();
        robot.update();

        double tractionAccel = slip.peakTractionAccel();

        while (opModeIsActive()) {
            telemetry.addData("axis", STRAFE ? "STRAFE" : "FORWARD");
            telemetry.addData("travelled (in)", "%.0f", travel);
            telemetry.addData("peak wheel/ground ratio", "%.2f", peakRatio);
            telemetry.addData("no-slip baseline ratio", "%.2f%s", baselineRatio,
                    baselineN == 0 ? "  (not sampled)" : "");
            telemetry.addLine();

            if (baselineN > 0 && baselineRatio > SLIP_RATIO - 0.05) {
                telemetry.addLine(String.format(Locale.US,
                        "WARNING: baseline %.2f is at or above SLIP_RATIO %.2f.",
                        baselineRatio, SLIP_RATIO));
                telemetry.addLine(String.format(Locale.US,
                        "  -> set SLIP_RATIO = %.2f and re-run. Common on the strafe axis,",
                        baselineRatio * 1.15));
                telemetry.addLine("  where the rollers scrub sideways even with perfect grip.");
                telemetry.addLine();
            }

            if (!everSlipped) {
                telemetry.addLine("NEVER SLIPPED -- the tyres held all the way to full power.");
                telemetry.addLine(String.format(Locale.US,
                        "  %s = %.0f is a floor, not the traction limit: the motors ran out",
                        STRAFE ? "A_STRAFE" : "A_FORWARD", tractionAccel));
                telemetry.addLine("  before the tyres did. Plan against it anyway -- you cannot");
                telemetry.addLine("  use acceleration the motors cannot produce.");
            } else {
                telemetry.addLine(String.format(Locale.US,
                        "Broke traction at power %.2f.", powerAtSlip));
                telemetry.addLine(String.format(Locale.US,
                        "  %s = %.0f   <- the real traction limit, in/s^2",
                        STRAFE ? "A_STRAFE" : "A_FORWARD", tractionAccel));
                telemetry.addLine("  Largest acceleration reached while NOT slipping.");
            }
            telemetry.addLine();
            telemetry.addLine(STRAFE
                    ? "Now set STRAFE = false and run again for A_FORWARD."
                    : "Now set STRAFE = true and run again for A_STRAFE.");
            telemetry.addLine("Then put the flywheel encoder back.");
            telemetry.update();
            robot.update();
        }
    }
}
