package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow;

import org.firstinspires.ftc.teamcode.utils.RateEstimator;
import org.firstinspires.ftc.teamcode.utils.Vector2;

public final class SlipDetector {
    private final double ratioThreshold;
    private final double minSpeed;
    private final double filterAlpha;

    private double filteredRatio = 1.0;
    private double slipTime = 0;
    private boolean slipping = false;
    private double peakCleanAccel = 0;
    private final RateEstimator cleanAccel = new RateEstimator();

    public SlipDetector() {
        this(1.18, 3.0, 0.25);
    }

    public SlipDetector(double ratioThreshold, double minSpeed, double filterAlpha) {
        this.ratioThreshold = ratioThreshold;
        this.minSpeed = minSpeed;
        this.filterAlpha = filterAlpha;
    }

    public void update(Vector2 wheelVelocity, Vector2 odometryVelocity, double dt) {
        double wheelSpeed = wheelVelocity.mag();
        double odoSpeed = odometryVelocity.mag();

        if (wheelSpeed < minSpeed) {
            filteredRatio += (1.0 - filteredRatio) * filterAlpha;
            slipTime = 0;
            slipping = false;
            cleanAccel.observeOnly(odoSpeed);
            return;
        }

        double ratio = wheelSpeed / Math.max(odoSpeed, 1e-3);
        filteredRatio += (ratio - filteredRatio) * filterAlpha;

        if (filteredRatio > ratioThreshold) {
            slipTime += dt;
            slipping = slipTime > 0.06;
            cleanAccel.invalidate();
        } else {
            slipTime = 0;
            slipping = false;
            double acc = cleanAccel.update(odoSpeed, dt);
            if (acc > peakCleanAccel) peakCleanAccel = acc;
        }
    }

    public boolean isSlipping() { return slipping; }

    public double slipRatio() { return filteredRatio; }

    public double peakTractionAccel() { return peakCleanAccel; }

    public void reset() {
        filteredRatio = 1.0;
        slipTime = 0;
        slipping = false;
        peakCleanAccel = 0;
        cleanAccel.reset();
    }
}
