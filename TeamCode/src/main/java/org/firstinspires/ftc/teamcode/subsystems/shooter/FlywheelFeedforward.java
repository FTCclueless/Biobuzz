package org.firstinspires.ftc.teamcode.subsystems.shooter;

import org.firstinspires.ftc.teamcode.utils.Lerp;

import java.util.Map;
import java.util.TreeMap;

public class FlywheelFeedforward {
    private final TreeMap<Double, Double> voltsAtVelocity = new TreeMap<>();

    public FlywheelFeedforward put(double velocity, double volts) {
        voltsAtVelocity.put(velocity, volts);
        return this;
    }

    public boolean isEmpty() { return voltsAtVelocity.isEmpty(); }

    public int size() { return voltsAtVelocity.size(); }

    public void clear() { voltsAtVelocity.clear(); }

    public double minVelocity() { return voltsAtVelocity.isEmpty() ? 0 : voltsAtVelocity.firstKey(); }

    public double maxVelocity() { return voltsAtVelocity.isEmpty() ? 0 : voltsAtVelocity.lastKey(); }

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
