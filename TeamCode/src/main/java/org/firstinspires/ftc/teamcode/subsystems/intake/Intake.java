package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    public static final String INTAKE_MOTOR = "intake";
    public static final String TRANSFER_MOTOR = "transfer";
    public static final String PIVOT_FRONT = "intakePivotFront";
    public static final String PIVOT_BACK = "intakePivotBack";

    public static final String FLOWER_SERVO = "flowerMechanism";
    public static final String LEFT_SENSOR = "leftEntranceSensor";
    public static final String RIGHT_SENSOR = "rightEntranceSensor";

    public enum Side { LEFT, RIGHT }

    public enum State {
        IDLE,
        INTAKE,
        FEED,
        EJECT,
        TEST
    }

    public enum BallColor { NONE, YELLOW, RED, BLUE }

    public static int CAPACITY = 4;
    public static int PRELOAD = 4;


    public static double EJECT_POWER = 0.7;

    public static double REJECT_POWER = 1.0;

    public static double intakePivotUpPosition = 0.6;
    public static double intakePivotDownPosition = 0.4;

    public static double backPivotUpPos = 0.7;
    public static double backPivotDownPos = 0.2;

    public static double rollerIntakePower = 1;
    public static double rollerFeedPower = 1;
    public static double transferIntakePower = 1; // placeholder
    public static double transferFeedPower = 1;
    /** Transfer power while the driver holds the nudge button: low enough not to jam balls into the closed blocker. */
    public static double transferNudgePower = 0.4;
    // Provisional normalized positions. Tune on the assembled mechanism.
    public static double flowerMechUpPosition = 0.15;
    public static double flowerMechDownPosition = 0.945;

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
    public final PriorityMotor intakeMotor, transferMotor;
    private final nPriorityServo flowerMechanismServo;
    public final nPriorityServo frontPivot, backPivot;
    private final EntranceSensor[] sensors = new EntranceSensor[2];
    //private final BallCounter counter;

    public State state = State.IDLE;
    public boolean overridePivotSide = true;
    public boolean pivotSideFront = true;
    private boolean reversed = false;
    /** Teleop sets this each loop while the nudge button is held; only acts when the transfer would otherwise be off. */
    public boolean nudgeTransfer = false;
    /** INTAKE also runs the transfer forward (teleop's first LB stage). Off = rollers only. */
    public boolean transferOnIntake = false;
    private int lastDirection = 0;

    private long rejectStart = -1;
    private String rejectReason = "";
    private Side rejectSide = null;

    private boolean flowerDown;

    public Intake(Robot robot) {
        this.robot = robot;

        intakeMotor = new PriorityMotor(
                new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR)},
                "intake", 2, 4, new double[]{-1}, robot.sensors);

        transferMotor = new PriorityMotor(
                new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, TRANSFER_MOTOR)},
                "transfer", 2, 4, new double[]{1}, robot.sensors);

        flowerMechanismServo = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, FLOWER_SERVO)},
                FLOWER_SERVO, nPriorityServo.ServoType.AXON_MINI_EXTENDED,
                0, 1, flowerMechUpPosition, new boolean[]{false}, 6, 7);

        frontPivot = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, PIVOT_FRONT)},
                "intakePivotFront", nPriorityServo.ServoType.AXON_MINI, 0,
                1, intakePivotUpPosition, new boolean[]{false}, 6, 7);
        backPivot = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, PIVOT_BACK)},
                "intakePivotBack", nPriorityServo.ServoType.AXON_MINI, 0,
                1, backPivotUpPos, new boolean[]{false}, 6, 7);

        robot.hardwareQueue.addDevices(intakeMotor, flowerMechanismServo, backPivot, frontPivot, transferMotor);

        sensors[Side.LEFT.ordinal()] = new EntranceSensor(robot, LEFT_SENSOR);
        sensors[Side.RIGHT.ordinal()] = new EntranceSensor(robot, RIGHT_SENSOR);

        int start = Globals.RUNMODE == RunMode.TELEOP ? Math.max(carriedCount, 0) : PRELOAD;
        //counter = new BallCounter(2, start, CAPACITY + 3, MIN_PRESENT_MS, MIN_GAP_MS);
    }

    public void update() {
        long now = System.currentTimeMillis();

        //counter.shot(robot.shooter.flywheel.consumeShots());
        for (Side s : Side.values()) {
            EntranceSensor es = sensors[s.ordinal()];
            es.read();
            //counter.update(s.ordinal(), es.present, lastDirection, now);
        }
        boolean intaking = state == State.INTAKE;
        if (!intaking) flowerDown = false;
        flowerMechanismServo.setTargetPos(flowerDown ? flowerMechDownPosition : flowerMechUpPosition);
        checkRejects(now, intaking);

        int direction = 0;
        switch (state) {
            case IDLE:
                intakeMotor.setTargetPower(0);
                transferMotor.setTargetPower(0);
                pivotDown(false);
                break;
            case INTAKE:
                direction = runIntake();
                break;
            case FEED:
                intakeMotor.setTargetPower(rollerFeedPower);
                transferMotor.setTargetPower(transferFeedPower);
                pivotDown(true);
                break;
            case EJECT:
                intakeMotor.setTargetPower(-EJECT_POWER);
                transferMotor.setTargetPower(-EJECT_POWER);

                pivotDown(true);
                direction = -1;
                break;
            case TEST:
                lastDirection = (int) Math.signum(intakeMotor.getPower());
                //carriedCount = counter.count();
                updateTelemetry();
                return;
        }

        if (nudgeTransfer && (state == State.IDLE || (state == State.INTAKE && !reversed && !rejecting()))) {
            transferMotor.setTargetPower(transferNudgePower);
        }

        lastDirection = direction;
        //carriedCount = counter.count();
        updateTelemetry();
    }

    private int runIntake() {
        if (rejecting()) {
            intakeMotor.setTargetPower(-REJECT_POWER);
            transferMotor.setTargetPower(-REJECT_POWER);

            pivotDown(true);
            return -1;
        }
        /*if (isFull()) {
            intakeMotor.setTargetPower(0);
            pivotDown(false);
            return 0;
        }*/
        pivotDown(true);
        intakeMotor.setTargetPower(rollerIntakePower * (reversed ? -1 : 1));
        transferMotor.setTargetPower(reversed ? -transferIntakePower : transferOnIntake ? transferIntakePower : 0);

        return reversed ? -1 : 1;
    }

    private boolean pivotSide()
    {
        return overridePivotSide ? pivotSideFront : false; // once custom pathing impl automatic pivot side
    }

    public void pivotDown(boolean down)
    {
        // Idle raises both sides. Reversing an active intake lowers both.
        boolean bothDown = down && (reversed || state == State.EJECT);
        boolean frontDown = down && (bothDown || pivotSide());
        boolean backDown = down && (bothDown || !pivotSide());
        frontPivot.setTargetPos(frontDown ? intakePivotDownPosition : intakePivotUpPosition);
        backPivot.setTargetPos(backDown ? backPivotDownPos : backPivotUpPos);
    }

    private void checkRejects(long now, boolean intaking) {
        return;
        /*if (!intaking) {
            rejectStart = -1;
            return;

        }
        if (rejecting()) {
            long t = now - rejectStart;
            boolean clear = rejectSide != null ? !sensors[rejectSide.ordinal()].present
                    : !sensors[0].present && !sensors[1].present;
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
        if (counter.count() > CAPACITY) startReject(now, "over limit", null);*/
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

    public void intake() {
        if (state != State.INTAKE) flowerDown = false;
        state = State.INTAKE;
    }

    public void feed() { state = State.FEED; }

    public void eject() { state = State.EJECT; }

    public void stop() { state = State.IDLE; }

    public void toggleFlower() {
        if (state == State.INTAKE) flowerDown = !flowerDown;
    }

    public boolean isFlowerDown() { return state == State.INTAKE && flowerDown; }

    /*public int getBallCount() { return counter.count(); }

    public void setBallCount(int n) { counter.set(n); carriedCount = counter.count(); }

    public boolean isFull() { return counter.count() >= CAPACITY; }*/

    public boolean hasSensor(Side s) { return sensors[s.ordinal()].kind != EntranceSensor.Kind.MISSING; }

    public boolean isBallPresent(Side s) { return sensors[s.ordinal()].present; }

    public BallColor getBallColor(Side s) { return sensors[s.ordinal()].ball; }

    public double getSensorDistanceCm(Side s) { return sensors[s.ordinal()].distanceCm; }

    public String getSensorKind(Side s) { return sensors[s.ordinal()].kind.toString(); }

    public boolean hasColorSensor(Side s) { return sensors[s.ordinal()].colorSensor != null; }

    public void reqIntake(boolean req) { if (req) intake(); }

    public void reqShoot(boolean req) { if (req) feed(); }

    public void reqOff(boolean req) { if (req) stop(); }

    public void setRollerDirection(boolean reversed) { this.reversed = reversed; }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", state);
        //TelemetryUtil.packet.put("Intake : balls", counter.count());
        //TelemetryUtil.packet.put("Intake : full", isFull());
        TelemetryUtil.packet.put("Intake : rejecting", rejecting() ? rejectReason : "no");
        TelemetryUtil.packet.put("Intake : reversed", reversed);
        TelemetryUtil.packet.put("Intake : flower target", flowerMechanismServo.getTargetPos());
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
