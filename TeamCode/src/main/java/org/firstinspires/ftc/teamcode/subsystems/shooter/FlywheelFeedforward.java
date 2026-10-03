package org.firstinspires.ftc.teamcode.subsystems.shooter;

import org.firstinspires.ftc.teamcode.utils.Lerp;

import java.util.Map;
import java.util.TreeMap;

/**
 * Measured voltage needed to hold each flywheel speed: a lookup table, in the same spirit as
 * {@link ShotTable2}, for the same reason. The linear kV/kS model assumes volts rise in a
 * straight line with speed, but air drag on the wheel grows faster than that, so a straight
 * line fitted across the whole range is slightly wrong at both ends and worst exactly where
 * you shoot from distance.
 *
 * Fill it from the Flywheel Characterizer, which prints paste-ready lines. Between two entries
 * it interpolates; outside the range it extrapolates along the nearest pair, so a target above
 * your highest measured point still gets a sensible guess rather than a clamp.
 *
 * Voltages are what the wheel needs at a NOMINAL battery. Flywheel divides by the real battery
 * voltage, so the table stays valid as the pack drains.
 */
public class FlywheelFeedforward {
    private final TreeMap<Double, Double> voltsAtVelocity = new TreeMap<>();

    /** @param velocity in/s, @param volts steady-state volts to hold it */
    public FlywheelFeedforward put(double velocity, double volts) {
        voltsAtVelocity.put(velocity, volts);
        return this;
    }

    public boolean isEmpty() { return voltsAtVelocity.isEmpty(); }

    public int size() { return voltsAtVelocity.size(); }

    public void clear() { voltsAtVelocity.clear(); }

    /** Lowest and highest measured speeds, for telemetry warnings. */
    public double minVelocity() { return voltsAtVelocity.isEmpty() ? 0 : voltsAtVelocity.firstKey(); }

    public double maxVelocity() { return voltsAtVelocity.isEmpty() ? 0 : voltsAtVelocity.lastKey(); }

    /**
     * Volts to hold this speed. Returns NaN when the table is empty, which tells the caller to
     * fall back to the linear model.
     */
    public double volts(double velocity) {
        if (voltsAtVelocity.isEmpty()) return Double.NaN;
        if (voltsAtVelocity.size() == 1) return voltsAtVelocity.firstEntry().getValue();

        Map.Entry<Double, Double> low = voltsAtVelocity.floorEntry(velocity);
        Map.Entry<Double, Double> high = voltsAtVelocity.ceilingEntry(velocity);

        if (low != null && high != null) {
            if (low.getKey().equals(high.getKey())) return low.getValue();
            double t = (velocity - low.getKey()) / (high.getKey() - low.getKey());
            return Lerp.lerp(low.getValue(), high.getValue(), t);
        }

        // Off the end of the table: continue the slope of the nearest measured pair rather
        // than flat-lining, so an unexpectedly long shot is merely approximate, not starved.
        Map.Entry<Double, Double> a;
        Map.Entry<Double, Double> b;
        if (high == null) {
            b = voltsAtVelocity.lastEntry();
            a = voltsAtVelocity.lowerEntry(b.getKey());
        } else {
            a = voltsAtVelocity.firstEntry();
            b = voltsAtVelocity.higherEntry(a.getKey());
        }
        double slope = (b.getValue() - a.getValue()) / (b.getKey() - a.getKey());
        return a.getValue() + slope * (velocity - a.getKey());
    }
}
