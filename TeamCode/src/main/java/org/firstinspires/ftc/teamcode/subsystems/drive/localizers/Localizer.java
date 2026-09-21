package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayList;
@Config
public class Localizer {
    protected Sensors sensors;
    protected Drivetrain drivetrain;

    public Encoder[] encoders;
    protected long lastTime = System.nanoTime();

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    protected double odoHeading = 0;

    public Pose2d expected = new Pose2d(0, 0, 0);
    public Pose2d currentPose = new Pose2d(0,0,0);
    public Pose2d currentVel = new Pose2d(0,0,0);
    public Pose2d currentAccel = new Pose2d(0,0,0);
    public Pose2d relCurrentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentAccel = new Pose2d(0,0,0);
    public Pose2d currentPowerVector = new Pose2d(0,0,0);

    protected ConstantAccelMath constAccelMath = new ConstantAccelMath();

    protected ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    protected ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    protected ArrayList<Long> nanoTimes = new ArrayList<Long>();
    protected ArrayList<Pose2d> relVelHistory = new ArrayList<Pose2d>();

    protected double maxVel = 0.0;
    protected double startHeadingOffset = 0;
    protected String color;
    protected String expectedColor;

    public static double targetVelTimeEstimate = 0.2;
    public static double targetAccelTimeEstimate = 0.1;

    public Localizer(Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        this.sensors = sensors;
        this.drivetrain = drivetrain;
        this.color = color;
        this.expectedColor = expectedColor;

        encoders = new Encoder[3];

        double ticksToInches = 1.0 / (2000.0 / 32.0 / Math.PI * 25.4);
        double leftPodOffset = 3.391 ;
        double rightPodOffset = 3.42;
        encoders[0] = new Encoder(new Pose2d(0, leftPodOffset),  -1, ticksToInches);
        encoders[1] = new Encoder(new Pose2d(0, -rightPodOffset), 1, ticksToInches);
        encoders[2] = new Encoder(new Pose2d(0.582, 0), -1, ticksToInches);

        relHistory.add(new Pose2d(0,0,0));
        poseHistory.add(new Pose2d(0,0,0));
        relVelHistory.add(new Pose2d(0,0,0));
        nanoTimes.add(System.nanoTime());
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }

    double headingOffset = 0;
    public void setPose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        headingOffset += h - this.heading;
        currentPose = new Pose2d(x, y, h);
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(currentPose.x, currentPose.y, currentPose.heading);
    }

    public void setPoseEstimate(Pose2d pose2d) {
        setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        startHeadingOffset = pose2d.getHeading();
    }

    public Pose2d getRelativePoseVelocity() {
        return new Pose2d(relCurrentVel.x, relCurrentVel.y, relCurrentVel.heading);
    }

    public Pose2d getGlobalVelocity() { return currentVel.clone(); }

    public Pose2d getGlobalAccel() { return currentAccel.clone(); }

    double fidelity = 1E-8;

    public double relDeltaX;
    public double relDeltaY;
    public double distanceTraveled = 0;

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime-lastTime)/1.0E9;
        lastTime = currentTime;

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        relDeltaY = deltaBack - deltaHeading*backX;
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        Pose2d relDelta = new Pose2d(relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime,relDelta,currentPose);

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public Pose2d interpolatedPastPose;

    public void findPastInterpolatedPose(long poseTime) {
        int indexOfDesiredNanoTime = 0;

        for (long time : nanoTimes) {
            if (time > poseTime) {
                indexOfDesiredNanoTime++;
            } else {
                break;
            }
        }

        indexOfDesiredNanoTime = Math.min(indexOfDesiredNanoTime, nanoTimes.size()-1);

        Pose2d pastTimeRobotPose = poseHistory.get(indexOfDesiredNanoTime).clone();
        Pose2d pastTimeRobotPose2 = poseHistory.get(Math.max(0, indexOfDesiredNanoTime-1)).clone();

        if (indexOfDesiredNanoTime != 0) {
            Pose2d errorInPastPoses = new Pose2d(
                    pastTimeRobotPose2.x - pastTimeRobotPose.x,
                    pastTimeRobotPose2.y - pastTimeRobotPose.y,
                    Utils.headingClip(pastTimeRobotPose2.heading - pastTimeRobotPose.heading)
            );

            double timeWeight = (double) (poseTime - nanoTimes.get(indexOfDesiredNanoTime)) /
                    (double) (nanoTimes.get(Math.max(0, indexOfDesiredNanoTime - 1)) - nanoTimes.get(indexOfDesiredNanoTime));

            interpolatedPastPose = new Pose2d(
                    pastTimeRobotPose.x + errorInPastPoses.x * timeWeight,
                    pastTimeRobotPose.y + errorInPastPoses.y * timeWeight,
                    pastTimeRobotPose.heading + errorInPastPoses.heading * timeWeight
            );
        } else {
            interpolatedPastPose = pastTimeRobotPose;
        }
    }

    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4;
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    private int velCalcLastIndex;
    private Pose2d calcVel(ArrayList<Pose2d> history) {
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        velCalcLastIndex = 0;
        long start = !nanoTimes.isEmpty() ? nanoTimes.get(0) : 0;
        for (int i = 0; i < nanoTimes.size(); i++) {
            totalTime = (double)(start - nanoTimes.get(i)) / 1.0E9;
            if (totalTime <= targetVelTimeEstimate) {
                actualVelTime = totalTime;
                relDeltaXTotal += history.get(i).getX();
                relDeltaYTotal += history.get(i).getY();
                velCalcLastIndex = i;
            }
        }
        double distanceTime = actualVelTime;
        if (velCalcLastIndex + 1 < nanoTimes.size()) {
            distanceTime = (double) (nanoTimes.get(0) - nanoTimes.get(velCalcLastIndex + 1)) / 1.0E9;
        }
        if (actualVelTime == 0 || distanceTime <= 0) return new Pose2d(0, 0, 0);
        return new Pose2d(
            (relDeltaXTotal) / distanceTime,
            (relDeltaYTotal) / distanceTime,
            Utils.headingClip(poseHistory.get(0).getHeading() - poseHistory.get(velCalcLastIndex).getHeading()) / actualVelTime
        );
    }

    private int accelCalcLastIndex;
    private Pose2d calcAccel(ArrayList<Pose2d> history) {
        double actualAccelTime = 0;
        double relDeltaVelXTotal = 0;
        double relDeltaVelYTotal = 0;
        double totalTime = 0;
        accelCalcLastIndex = 0;
        long start = !nanoTimes.isEmpty() ? nanoTimes.get(0) : 0;

        for (int i = 0; i < nanoTimes.size()-1; i++) {
            totalTime = (double)(start - nanoTimes.get(i)) / 1.0E9;

            if (totalTime <= targetAccelTimeEstimate) {
                actualAccelTime = totalTime;

                double deltaVX = history.get(i).getX() - history.get(i + 1).getX();
                double deltaVY = history.get(i).getY() - history.get(i + 1).getY();

                relDeltaVelXTotal += deltaVX;
                relDeltaVelYTotal += deltaVY;
                accelCalcLastIndex = i;
            }
        }
        double accelSpan = actualAccelTime;
        if (accelCalcLastIndex + 1 < nanoTimes.size()) {
            accelSpan = (double) (nanoTimes.get(0) - nanoTimes.get(accelCalcLastIndex + 1)) / 1.0E9;
        }
        if (actualAccelTime == 0 || accelSpan <= 0) return new Pose2d(0, 0, 0);
        return new Pose2d(
            (relDeltaVelXTotal) / accelSpan,
            (relDeltaVelYTotal) / accelSpan
        );
    }

    public void updateVelocity() {
        relCurrentVel = calcVel(relHistory);
        currentVel = new Pose2d(
            relCurrentVel.x * Math.cos(heading) - relCurrentVel.y * Math.sin(heading),
            relCurrentVel.x * Math.sin(heading) + relCurrentVel.y * Math.cos(heading),
            relCurrentVel.heading
        );

        relVelHistory.add(0, relCurrentVel.clone());

        relCurrentAccel = calcAccel(relVelHistory);
        currentAccel = new Pose2d(
            relCurrentAccel.x * Math.cos(heading) - relCurrentAccel.y * Math.sin(heading),
            relCurrentAccel.x * Math.sin(heading) + relCurrentAccel.y * Math.cos(heading)
        );

        while (velCalcLastIndex + 1 < nanoTimes.size()) {
            nanoTimes.remove(nanoTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
            relVelHistory.remove(relVelHistory.size() - 1);
        }
    }

    double a = 0.0045;
    double b = -0.0728;
    double c = 0.8062;
    double d = Math.sqrt(c/a);

    protected void updateExpected() {
        double totalVel = Math.sqrt(Math.pow(currentVel.x, 2) + Math.pow(currentVel.y, 2));
        double distance = getExpectedDistance(totalVel);

        if (totalVel <= d) {
            distance = totalVel*(getExpectedDistance(d)/d);
        }

        if (totalVel <= 5) {
            expected.x = x;
            expected.y = y;
            return;
        }

        expected.x = x + distance * currentVel.x/totalVel;
        expected.y = y + distance * currentVel.y/totalVel;
        expected.heading = heading;
    }

    private double getExpectedDistance (double x) {
        return a*Math.pow(x,2) + b*x + c;
    }

    protected MovingAverage movingAverage = new MovingAverage(100);

    public double combineHeadings(double headingError) {
        movingAverage.addData(headingError);
        double averageError = movingAverage.getMovingAverageForNum();
        movingAverage.updateValsRetroactively(averageError);
        return movingAverage.getMovingAverageForNum();
    }

    public void updateField () {
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" distance", distanceTraveled);

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), color);
    }
}
