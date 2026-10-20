package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    public static final String INTAKE_MOTOR = "intake";
    public static final String INTAKE_MOTOR_2 = "intake2";
    public static final String LEFT_PIVOT = "leftIntakePivot";
    public static final String RIGHT_PIVOT = "rightIntakePivot";
    public static final String FLOWER_SERVO = "flowerMechanism";
    public static final String FLOWER_ENCODER = "intakeFlowerMechEncoder";
    public static final String LEFT_SENSOR = "leftEntranceSensor";
    public static final String RIGHT_SENSOR = "rightEntranceSensor";

    public enum Side { LEFT, RIGHT }

    public enum State {
        IDLE,
        INTAKE,
        FEED,
        EJECT,
        POLLEN_RETRIEVAL_START,
        POLLEN_RETRIEVAL_INTAKE,
        POLLEN_RETRIEVAL_END,
        TEST
    }

    public enum BallColor { NONE, YELLOW, RED, BLUE }

    public static int CAPACITY = 4;
    public static int PRELOAD = 4;

    public static double intakeRollerPower = 1.0;
    public static double shootRollerPower = 1.0;
    public static double EJECT_POWER = 0.7;
    public static double REJECT_POWER = 1.0;
    public static boolean LIFT_WHILE_FEEDING = true;

    public static double intakePivotUpPosition = 0.0;
    public static double intakePivotDownPosition = 1.6;
    public static double SIDE_DEADBAND = 0.05;

    public static double flowerMechDownPosition = 4.5;
    public static double flowerMechUpPosition = 0.0;
    public static double flowerMechEncoderMaxVoltage = 3.3;
    public static double flowerMechPositionThreshold = 0.1;
    public static PID flowerMechPID = new PID(1, 0, 0);

    public static double PRESENT_CM = 4.0;
    public static boolean BEAM_ACTIVE_LOW = true;
    public static long MIN_PRESENT_MS = 30;
    public static long MIN_GAP_MS = 40;
    public static long REJECT_MIN_MS = 250;
    public static long REJECT_MAX_MS = 800;

    public static boolean USE_COLOR = true;
    public static double YELLOW_HUE_MIN = 35, YELLOW_HUE_MAX = 75;
    public static double BLUE_HUE_MIN = 190, BLUE_HUE_MAX = 250;
    public static double RED_HUE_BELOW = 20, RED_HUE_ABOVE = 335;
    public static double MIN_SATURATION = 0.35;

    private static int carriedCount = -1;

    private final Robot robot;
    public final PriorityMotor intakeMotor;
    public final PriorityCRServo flowerMechanismServo;
    public final nPriorityServo lintake, rintake;
    private final AnalogInput flowerEncoder;
    private final EntranceSensor[] sensors = new EntranceSensor[2];
    private final BallCounter counter;

    public State state = State.IDLE;
    private Side side = Side.LEFT;
    private boolean reversed = false;
    private int lastDirection = 0;

    private long rejectStart = -1;
    private String rejectReason = "";
    private Side rejectSide = Side.LEFT;

    private double flowerAngle = 0;
    private double lastFlowerRaw = 0;

    public Intake(Robot robot) {
        this.robot = robot;

        intakeMotor = new PriorityMotor(
                new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR),
                        robot.hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_2)},
                "intake", 2, 4, new double[]{1, 1}, robot.sensors);

        flowerMechanismServo = new PriorityCRServo(
                new CRServo[]{robot.hardwareMap.get(CRServo.class, FLOWER_SERVO)},
                "flowerMechanism", PriorityCRServo.ServoType.AXON_MINI, new boolean[]{false}, 6, 7);

        lintake = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, LEFT_PIVOT)},
                "leftIntakePivot", nPriorityServo.ServoType.AXON_MINI, intakePivotUpPosition,
                intakePivotDownPosition, intakePivotUpPosition, new boolean[]{false}, 6, 7);
        rintake = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, RIGHT_PIVOT)},
                "rightIntakePivot", nPriorityServo.ServoType.AXON_MINI, intakePivotUpPosition,
                intakePivotDownPosition, intakePivotUpPosition, new boolean[]{false}, 6, 7);

        robot.hardwareQueue.addDevices(intakeMotor, flowerMechanismServo, lintake, rintake);

        flowerEncoder = robot.hardwareMap.get(AnalogInput.class, FLOWER_ENCODER);
        lastFlowerRaw = flowerRaw();
        flowerAngle = lastFlowerRaw;

        sensors[Side.LEFT.ordinal()] = new EntranceSensor(robot, LEFT_SENSOR);
        sensors[Side.RIGHT.ordinal()] = new EntranceSensor(robot, RIGHT_SENSOR);

        int start = Globals.RUNMODE == RunMode.TELEOP ? Math.max(carriedCount, 0) : PRELOAD;
        counter = new BallCounter(2, start, CAPACITY + 3, MIN_PRESENT_MS, MIN_GAP_MS);
    }

    public void update() {
        long now = System.currentTimeMillis();

        counter.shot(robot.shooter.flywheel.consumeShots());
        for (Side s : Side.values()) {
            EntranceSensor es = sensors[s.ordinal()];
            es.read();
            counter.update(s.ordinal(), es.present, lastDirection, now);
        }
        readFlower();

        boolean intaking = state == State.INTAKE || state == State.POLLEN_RETRIEVAL_INTAKE;
        if (intaking) chooseSide();
        checkRejects(now, intaking);

        int direction = 0;
        switch (state) {
            case IDLE:
                intakeMotor.setTargetPower(0);
                flowerMechanismServo.setTargetPower(0);
                pivots(false, false);
                break;
            case INTAKE:
                direction = runIntake();
                break;
            case FEED:
                intakeMotor.setTargetPower(shootRollerPower);
                pivots(!LIFT_WHILE_FEEDING, !LIFT_WHILE_FEEDING);
                break;
            case EJECT:
                intakeMotor.setTargetPower(-EJECT_POWER);
                pivots(side == Side.LEFT, side == Side.RIGHT);
                direction = -1;
                break;
            case POLLEN_RETRIEVAL_START:
                intakeMotor.setTargetPower(0);
                pivots(false, false);
                if (driveFlowerTo(flowerMechDownPosition)) state = State.POLLEN_RETRIEVAL_INTAKE;
                break;
            case POLLEN_RETRIEVAL_INTAKE:
                flowerMechanismServo.setTargetPower(0);
                direction = runIntake();
                break;
            case POLLEN_RETRIEVAL_END:
                intakeMotor.setTargetPower(0);
                pivots(false, false);
                if (driveFlowerTo(flowerMechUpPosition)) state = State.IDLE;
                break;
            case TEST:
                updateTelemetry();
                return;
        }

        lastDirection = direction;
        carriedCount = counter.count();
        updateTelemetry();
    }

    private int runIntake() {
        if (rejecting()) {
            intakeMotor.setTargetPower(-REJECT_POWER);
            pivots(rejectSide == Side.LEFT, rejectSide == Side.RIGHT);
            return -1;
        }
        if (isFull()) {
            intakeMotor.setTargetPower(0);
            pivots(false, false);
            return 0;
        }
        pivots(side == Side.LEFT, side == Side.RIGHT);
        intakeMotor.setTargetPowerSmooth(intakeRollerPower * (reversed ? -1 : 1), 0.1);
        return reversed ? -1 : 1;
    }

    private void pivots(boolean leftDown, boolean rightDown) {
        lintake.setTargetAngle(leftDown ? intakePivotDownPosition : intakePivotUpPosition);
        rintake.setTargetAngle(rightDown ? intakePivotDownPosition : intakePivotUpPosition);
    }

    private void chooseSide() {
        if (rejecting()) return;
        double y = robot.drivetrain.state == Drivetrain.State.FOLLOW_TRAJECTORY
                ? ROBOT_VELOCITY.y / DriveConstants.V_STRAFE
                : robot.drivetrain.getLastMoveVector().y;
        if (y > SIDE_DEADBAND) side = Side.RIGHT;
        else if (y < -SIDE_DEADBAND) side = Side.LEFT;
    }

    private void checkRejects(long now, boolean intaking) {
        if (!intaking) {
            rejectStart = -1;
            return;
        }
        if (rejecting()) {
            long t = now - rejectStart;
            boolean clear = !sensors[rejectSide.ordinal()].present;
            boolean done = t >= REJECT_MIN_MS && clear
                    && (!"over limit".equals(rejectReason) || counter.count() <= CAPACITY);
            if (done || t >= REJECT_MAX_MS) rejectStart = -1;
            return;
        }
        for (Side s : Side.values()) {
            EntranceSensor es = sensors[s.ordinal()];
            if (counter.count() > CAPACITY && es.present) {
                startReject(now, "over limit", s);
                return;
            }
            if (isOpponentNectar(es.ball)) {
                startReject(now, "opponent nectar", s);
                return;
            }
        }
        if (counter.count() > CAPACITY) startReject(now, "over limit", side);
    }

    private void startReject(long now, String reason, Side s) {
        rejectStart = now;
        rejectReason = reason;
        rejectSide = s;
    }

    private boolean rejecting() { return rejectStart >= 0; }

    private static boolean isOpponentNectar(BallColor c) {
        return (Globals.isRed && c == BallColor.BLUE) || (!Globals.isRed && c == BallColor.RED);
    }

    private double flowerRaw() {
        return flowerEncoder.getVoltage() / flowerMechEncoderMaxVoltage * 2 * Math.PI;
    }

    private void readFlower() {
        double raw = flowerRaw();
        double d = raw - lastFlowerRaw;
        flowerAngle += Math.atan2(Math.sin(d), Math.cos(d));
        lastFlowerRaw = raw;
    }

    private boolean driveFlowerTo(double target) {
        double err = target - flowerAngle;
        if (Math.abs(err) < flowerMechPositionThreshold) {
            flowerMechanismServo.setTargetPower(0);
            return true;
        }
        flowerMechanismServo.setTargetPower(flowerMechPID.update(err, -1.0, 1.0));
        return false;
    }

    public void intake() { state = State.INTAKE; }

    public void intake(Side side) { this.side = side; state = State.INTAKE; }

    public void feed() { state = State.FEED; }

    public void eject() { state = State.EJECT; }

    public void stop() {
        if (state == State.POLLEN_RETRIEVAL_START || state == State.POLLEN_RETRIEVAL_INTAKE) {
            state = State.POLLEN_RETRIEVAL_END;
        } else if (state != State.POLLEN_RETRIEVAL_END) {
            state = State.IDLE;
        }
    }

    public void startPollenRetrieval() {
        lastFlowerRaw = flowerRaw();
        flowerAngle = lastFlowerRaw;
        state = State.POLLEN_RETRIEVAL_START;
    }

    public void setSide(Side side) { this.side = side; }

    public Side getSide() { return side; }

    public int getBallCount() { return counter.count(); }

    public void setBallCount(int n) { counter.set(n); carriedCount = counter.count(); }

    public boolean isFull() { return counter.count() >= CAPACITY; }

    public boolean hasSensor(Side s) { return sensors[s.ordinal()].kind != EntranceSensor.Kind.MISSING; }

    public boolean isBallPresent(Side s) { return sensors[s.ordinal()].present; }

    public BallColor getBallColor(Side s) { return sensors[s.ordinal()].ball; }

    public double getSensorDistanceCm(Side s) { return sensors[s.ordinal()].distanceCm; }

    public String getSensorKind(Side s) { return sensors[s.ordinal()].kind.toString(); }

    public boolean hasColorSensor(Side s) { return sensors[s.ordinal()].colorSensor != null; }

    public double getFlowerAngle() { return flowerAngle; }

    public void reqIntake(boolean req) { if (req) intake(); }

    public void reqPollenRetrieval(boolean req) { if (req) startPollenRetrieval(); }

    public void reqShoot(boolean req) { if (req) feed(); }

    public void reqOff(boolean req) { if (req) stop(); }

    public void setRollerDirection(boolean reversed) { this.reversed = reversed; }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", state);
        TelemetryUtil.packet.put("Intake : side", side);
        TelemetryUtil.packet.put("Intake : balls", counter.count());
        TelemetryUtil.packet.put("Intake : full", isFull());
        TelemetryUtil.packet.put("Intake : rejecting", rejecting() ? rejectReason : "no");
        TelemetryUtil.packet.put("Intake : reversed", reversed);
        TelemetryUtil.packet.put("Intake : flower angle (rad)", flowerAngle);
        LogUtil.intakeState.set(state.toString());
        LogUtil.intakeReversed.set(reversed);
    }

    private static final class EntranceSensor {
        enum Kind { DISTANCE, BEAM, MISSING }

        final Kind kind;
        private final DistanceSensor distance;
        private final NormalizedColorSensor colorSensor;
        private final DigitalChannel beam;

        boolean present = false;
        BallColor ball = BallColor.NONE;
        double distanceCm = Double.NaN;

        EntranceSensor(Robot robot, String name) {
            DistanceSensor d = robot.hardwareMap.tryGet(DistanceSensor.class, name);
            DigitalChannel b = d == null ? robot.hardwareMap.tryGet(DigitalChannel.class, name) : null;
            distance = d;
            beam = b;
            colorSensor = d instanceof NormalizedColorSensor ? (NormalizedColorSensor) d : null;
            if (b != null) b.setMode(DigitalChannel.Mode.INPUT);
            kind = d != null ? Kind.DISTANCE : b != null ? Kind.BEAM : Kind.MISSING;
        }

        void read() {
            switch (kind) {
                case DISTANCE:
                    distanceCm = distance.getDistance(DistanceUnit.CM);
                    present = distanceCm < PRESENT_CM;
                    break;
                case BEAM:
                    present = beam.getState() != BEAM_ACTIVE_LOW;
                    break;
                default:
                    present = false;
            }
            ball = present && USE_COLOR && colorSensor != null ? classify() : BallColor.NONE;
        }

        private BallColor classify() {
            NormalizedRGBA c = colorSensor.getNormalizedColors();
            float[] hsv = new float[3];
            Color.RGBToHSV((int) (Math.min(1, c.red) * 255), (int) (Math.min(1, c.green) * 255),
                    (int) (Math.min(1, c.blue) * 255), hsv);
            double hue = hsv[0], sat = hsv[1];
            if (sat < MIN_SATURATION) return BallColor.NONE;
            if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX) return BallColor.YELLOW;
            if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) return BallColor.BLUE;
            if (hue <= RED_HUE_BELOW || hue >= RED_HUE_ABOVE) return BallColor.RED;
            return BallColor.NONE;
        }
    }
}
