package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;

public final class FollowMetrics {
    private static final double WEAVE_DEADBAND = 0.15;
    private static final double ACCEL_THRESHOLD = 10.0;
    public static final double WEAVE_LIMIT_HZ = 1.5;

    private double peakContour, peakSaturation;
    private double sumContour, sumContourSq;
    private int n, crossings, saturatedLoops;
    private double lastSign, elapsed;
    private double lagAccelSum, lagCruiseSum;
    private int lagAccelN, lagCruiseN;

    public void observe(PathFollower.Command cmd, Trajectory traj, double dt) {
        n++;
        elapsed += dt;

        double c = cmd.contourError;
        peakContour = Math.max(peakContour, Math.abs(c));
        sumContour += c;
        sumContourSq += c * c;

        if (Math.abs(c) > WEAVE_DEADBAND) {
            double sign = Math.signum(c);
            if (lastSign != 0 && sign != lastSign) crossings++;
            lastSign = sign;
        }

        peakSaturation = Math.max(peakSaturation, cmd.saturation);
        if (cmd.saturation > 1.0) saturatedLoops++;

        double aRef = Math.abs(traj.profile().plannedAccel(cmd.s));
        if (aRef > ACCEL_THRESHOLD) { lagAccelSum += Math.abs(cmd.lagError); lagAccelN++; }
        else                        { lagCruiseSum += Math.abs(cmd.lagError); lagCruiseN++; }
    }

    public int samples()               { return n; }
    public double peakContour()        { return peakContour; }
    public double peakSaturation()     { return peakSaturation; }
    public double rmsContour()         { return n == 0 ? 0 : Math.sqrt(sumContourSq / n); }
    public double meanContour()        { return n == 0 ? 0 : sumContour / n; }
    public double weaveHz()            { return elapsed <= 0 ? 0 : crossings / elapsed; }
    public double saturatedFraction()  { return n == 0 ? 0 : (double) saturatedLoops / n; }
    public double meanLagAccel()       { return lagAccelN == 0 ? 0 : lagAccelSum / lagAccelN; }
    public double meanLagCruise()      { return lagCruiseN == 0 ? 0 : lagCruiseSum / lagCruiseN; }

    public boolean oscillating() { return weaveHz() > WEAVE_LIMIT_HZ; }

    public double cost() { return rmsContour() + 0.5 * peakContour(); }

    public String summary() {
        return String.format(java.util.Locale.US,
                "rms %.2f  peak %.2f  bias %+.2f  weave %.1f/s  sat %.0f%%",
                rmsContour(), peakContour(), meanContour(), weaveHz(), 100 * saturatedFraction());
    }
}
