package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    private final Robot robot;
    public final PriorityMotor intakeMotor;
    public final PriorityCRServo flowerMechanismServo;
    public final nPriorityServo lintake, rintake;
    //public final nPriorityServo rindex, lindex;

    private double dFlowerMechEncoder = 0.0;
    private double lastFlowerMechEncoder = 0.0;

    private boolean requestIntake = false, requestShoot = false, requestOff = false, requestPollenRetrieval = false, reversed = false;

    public static double intakeRollerPower = 1.0, intakeFeedPower = 0.4, shootRollerPower = 1.0, shootFeedPower = 0.8;

    public static double flowerMechDownPosition = 4.5;
    public static double flowerMechUpPosition = 0.0;
    public static double flowerMechEncoderMaxVoltage = 3.3;
    public static double flowerMechPositionThreshold = 0.1;
    public static PID flowerMechPID = new PID(1, 0, 0); // TODO: Tune

    public static double intakePivotUpPosition = 0.0;
    public static double intakePivotDownPosition = 1.6;

    public static long intakeReverseDuration = 200;

    public enum State {
        IDLE,
        INTAKE,
        POLLEN_RETRIEVAL_START,
        POLLEN_RETRIEVAL_INTAKE,
        POLLEN_RETRIEVAL_END,
        TEST
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;


        intakeMotor = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "intake"), robot.hardwareMap.get(DcMotorEx.class, "intake2")},
            "intake", 2, 4,
            new double[] {1, 1}, robot.sensors
        );

        flowerMechanismServo = new PriorityCRServo(new CRServo[]{robot.hardwareMap.get(CRServo.class, "flowerMechanism")},
                "flowerMechanism", PriorityCRServo.ServoType.AXON_MINI,
                new boolean[] {false},
                6, 7
        );

        lintake = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "leftIntakePivot")}, "leftIntakePivot", nPriorityServo.ServoType.AXON_MINI,
                intakePivotUpPosition, intakePivotDownPosition, intakePivotUpPosition, new boolean[] {false}, 6, 7);
        rintake = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "rightIntakePivot")}, "rightIntakePivot", nPriorityServo.ServoType.AXON_MINI,
                intakePivotUpPosition, intakePivotDownPosition, intakePivotUpPosition, new boolean[] {false}, 6, 7);
        robot.hardwareQueue.addDevices(intakeMotor, flowerMechanismServo, lintake, rintake);
    }

    private double encoderVoltageToRad(double v)
    {
        return (v/flowerMechEncoderMaxVoltage)*2*Math.PI;
    }

    private void runIntake()
    {
        boolean forward = Math.signum(robot.drivetrain.getLastMoveVector().y) == 1.0;

        if(forward)
        {
            rintake.setTargetAngle(intakePivotDownPosition);
            lintake.setTargetAngle(intakePivotUpPosition);
        }
        else
        {
            lintake.setTargetAngle(intakePivotDownPosition);
            rintake.setTargetAngle(intakePivotUpPosition);
        }
        intakeMotor.setTargetPowerSmooth(intakeRollerPower * (reversed ? -1 : 1), 0.1);
    }

    private void goToTargetFlowerMech(boolean up, State target)
    {
        double targetPos = up ? flowerMechUpPosition : flowerMechDownPosition;
        double resetSign = up ? -1.0 : 1.0;
        boolean resetExpr = resetSign == -1.0 ? dFlowerMechEncoder > 0.0 : dFlowerMechEncoder < 0.0;

        dFlowerMechEncoder = encoderVoltageToRad(robot.sensors.getFlowerMechanismEncoderVoltageRaw())-lastFlowerMechEncoder;

        if(resetExpr)
        {
            // the encoder reset
            dFlowerMechEncoder += 2*Math.PI*resetSign;
        }

        lastFlowerMechEncoder += dFlowerMechEncoder;
        double err = targetPos-lastFlowerMechEncoder;

        if(Math.abs(err) < flowerMechPositionThreshold)
        {
            state = target;
            return;
        }

        double power = flowerMechPID.update(err, -1.0, 1.0);
        flowerMechanismServo.setTargetPower(power);
    }

    public void update() {
        switch (state) {
            case IDLE: {
                requestOff = false;

                intakeMotor.setTargetPower(0.0);
                flowerMechanismServo.setTargetPower(0.0);
                lintake.setTargetAngle(intakePivotUpPosition);
                rintake.setTargetAngle(intakePivotUpPosition);

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                if(requestPollenRetrieval) {
                    requestPollenRetrieval = false;
                    state = State.POLLEN_RETRIEVAL_START;
                    lastFlowerMechEncoder = encoderVoltageToRad(robot.sensors.getFlowerMechanismEncoderVoltageRaw()); // Assume the encoder starts at 0 every time
                }

                break;
            }
            case INTAKE: {
                requestIntake = false;
                runIntake();

                if(requestPollenRetrieval) {
                    requestPollenRetrieval = false;
                    state = State.POLLEN_RETRIEVAL_START;
                    lastFlowerMechEncoder = encoderVoltageToRad(robot.sensors.getFlowerMechanismEncoderVoltageRaw()); // Assume the encoder starts at 0 every time
                }

                if (requestOff) {
                    state = State.IDLE;
                    requestIntake = false;
                    requestOff = false;
                    requestShoot = false;
                    requestPollenRetrieval = false;
                }

                break;
            }
            case POLLEN_RETRIEVAL_START: {
                requestPollenRetrieval = false;
                goToTargetFlowerMech(false, State.POLLEN_RETRIEVAL_INTAKE);
                break;
            }
            case POLLEN_RETRIEVAL_INTAKE: {
                runIntake();
                if (requestOff) {
                    state = State.POLLEN_RETRIEVAL_END;
                }
            }
            case POLLEN_RETRIEVAL_END: {
                goToTargetFlowerMech(true, State.IDLE);
                requestIntake = false;
                requestOff = false;
                requestShoot = false;
                requestPollenRetrieval = false;
                break;
            }
            case TEST: {
                break;
            }
        }

        this.updateTelemetry();
    }

    public void reqIntake (boolean req) {
        requestIntake = req;
    }

    public void reqPollenRetrieval(boolean req) {
        requestPollenRetrieval = req;
    }

    public void reqShoot (boolean req) {
        requestShoot = req;
    }

    public void reqOff (boolean req) { requestOff = req; }

    public void setRollerDirection(boolean reversed) { this.reversed = reversed; }

    /*public void setRightBlocker(boolean on) {
        rindex.setTargetAngle(on ? 0.19 : 0.05);
    }*/

    /*public void setLeftBlocker(boolean on) {
        lindex.setTargetAngle(on ? 0.61 : 0.9);
    }*/

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
        TelemetryUtil.packet.put("Intake : reversed", reversed);
        LogUtil.intakeReversed.set(reversed);
    }
}
