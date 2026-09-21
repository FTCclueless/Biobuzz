package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class ConstantAccelMath {
    public static final double FIDELITY = 1E-6;

    private double lastLoop = 0.008;
    private Pose2d lastRelativeDelta = new Pose2d(0,0,0);

    public void calculate(double loopTime, Pose2d relDelta, Pose2d currPose){
        if (loopTime <= 1e-9 || lastLoop <= 1e-9) {
            lastRelativeDelta = relDelta.clone();
            if (loopTime > 1e-9) lastLoop = loopTime;
            return;
        }

        double relDeltaX = relDelta.x;
        double relDeltaY = relDelta.y;
        double deltaHeading = relDelta.heading;

        double arx = (relDeltaX*lastLoop - lastRelativeDelta.x*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrx = relDeltaX/loopTime - arx*loopTime;
        double ary = (relDeltaY*lastLoop - lastRelativeDelta.y*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vry = relDeltaY/loopTime - ary*loopTime;
        double arh = (deltaHeading*lastLoop - lastRelativeDelta.heading*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrh = deltaHeading/loopTime - arh*loopTime;

        AdaptiveQuadrature xQuadrature = new AdaptiveQuadrature(new double[] {vrx,2*arx},new double[] {currPose.heading,vrh,arh});
        AdaptiveQuadrature yQuadrature = new AdaptiveQuadrature(new double[] {vry,2*ary},new double[] {currPose.heading,vrh,arh});

        currPose.x += xQuadrature.evaluateCos(FIDELITY, 0, loopTime, 0) - yQuadrature.evaluateSin(FIDELITY, 0, loopTime, 0);
        currPose.y += yQuadrature.evaluateCos(FIDELITY, 0, loopTime, 0) + xQuadrature.evaluateSin(FIDELITY, 0, loopTime, 0);
        currPose.heading += deltaHeading;

        lastRelativeDelta = relDelta.clone();
        lastLoop = loopTime;
    }
}
