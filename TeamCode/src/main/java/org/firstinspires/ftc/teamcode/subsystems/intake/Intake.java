package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    private final Robot robot;
    public final PriorityMotor intakeMotor;
    public final nPriorityServo flowerMechanismServo;
    //public final nPriorityServo rindex, lindex;

    private boolean requestIntake = false, requestShoot = false, requestOff = false, requestPollenRetrieval = false, reversed = false;

    public static double intakeRollerPower = 1.0, intakeFeedPower = 0.4, shootRollerPower = 1.0, shootFeedPower = 0.8;
    public static double flowerMechMaxPos = 0.85, flowerMechMinPos = 0.10; // placeholder
    public static double flowerMechDownPos = flowerMechMinPos;
    public static double flowerMechUpPos = flowerMechMaxPos;
    public static long intakeReverseDuration = 200;

    public enum State {
        IDLE,
        INTAKE,
        POLLEN_RETRIEVAL,
        TEST
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;


        intakeMotor = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "intake")},
            "intake", 2, 4,
            new double[] {1}, robot.sensors
        );

        flowerMechanismServo = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "flowerMechanism")},
                "flowerMechanism", nPriorityServo.ServoType.AXON_MINI,
                flowerMechMinPos, flowerMechMaxPos, flowerMechUpPos,
                new boolean[] {false},
                6, 7
        );

        robot.hardwareQueue.addDevices(intakeMotor);
    }

    long turnedOffTime = 0;

    public void update() {
        setRollerDirection(Math.signum(this.robot.drivetrain.getLastMoveVector().y) == -1.0);
        switch (state) {
            case IDLE: {
                requestOff = false;

                intakeMotor.setTargetPower(0.0);
                flowerMechanismServo.setTargetAngle(flowerMechUpPos);

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                if(requestPollenRetrieval) {
                    requestPollenRetrieval = false;
                    state = State.POLLEN_RETRIEVAL;
                }

                break;
            }
            case INTAKE: {
                requestIntake = false;

                intakeMotor.setTargetPowerSmooth(intakeRollerPower * (reversed ? -1 : 1), 0.1);

                if(requestPollenRetrieval) {
                    requestPollenRetrieval = false;
                    state = State.POLLEN_RETRIEVAL;
                }

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

                break;
            }
            case POLLEN_RETRIEVAL: {
                requestPollenRetrieval = false;
                intakeMotor.setTargetPowerSmooth(intakeRollerPower * (reversed ? -1 : 1), 0.1);
                flowerMechanismServo.setTargetAngle(flowerMechDownPos);

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

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
