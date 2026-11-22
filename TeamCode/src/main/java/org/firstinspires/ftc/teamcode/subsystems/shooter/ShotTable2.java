package org.firstinspires.ftc.teamcode.subsystems.shooter;

import org.firstinspires.ftc.teamcode.utils.Lerp;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

public class ShotTable2 {
    private final TreeMap<Double, ShotPoint> table = new TreeMap<>(); // distance -> flywheel

    public ShotTable2() {
        // TODO Find new values. These are placeholders.
        table.put( 32.77, new ShotPoint(420)
                .addValue(410, Math.toRadians(55))
        );

        table.put( 40.0, new ShotPoint(440)
            .addValue(430, Math.toRadians(55))
        );

        table.put( 50.0, new ShotPoint(460)
            .addValue(440, Math.toRadians(55))
        );


    }

    public double getFlywheelForDistance(double distance) {
        return tableLerp(table, distance, (ShotPoint shotPoint) -> {
            if (shotPoint == null) return 0.0;
            return shotPoint.flywheelTargetVel;
        });
    }

    public double getLaunchAngleForDistanceAndFlywheel(double distance, double flywheelVel) {
        return tableLerp(table, distance, (ShotPoint shotPoint) -> {
            if (shotPoint == null) return 0.0;
            return tableLerp(shotPoint.launchTable, flywheelVel, (Double launchAngle) -> {
                if (launchAngle == null) return 0.0;
                return launchAngle;
            });
        });
    }

    private static <E> double tableLerp(TreeMap<Double, E> table, Double key, Function<E, Double> mapFunction) {
        Map.Entry<Double, E> lowEntry = table.floorEntry(key);
        Map.Entry<Double, E> highEntry = table.ceilingEntry(key);
        if (lowEntry == null && highEntry == null) return mapFunction.apply(null);
        if (lowEntry == null) return mapFunction.apply(highEntry.getValue());
        if (highEntry == null) return mapFunction.apply(lowEntry.getValue());
        if (lowEntry.getKey().equals(highEntry.getKey())) return mapFunction.apply(lowEntry.getValue());
        return Lerp.lerp(
            mapFunction.apply(lowEntry.getValue()),
            mapFunction.apply(highEntry.getValue()),
            (key - lowEntry.getKey()) / (highEntry.getKey() - lowEntry.getKey())
        );
    }

    private static class ShotPoint {
        private final double flywheelTargetVel;
        private final TreeMap<Double, Double> launchTable = new TreeMap<>(); // flywheel -> launch angle

        private ShotPoint(double flywheelTargetVel) {
            this.flywheelTargetVel = flywheelTargetVel;
        }

        private ShotPoint addValue(double flywheelVel, double launchAngle) {
            launchTable.put(flywheelVel, launchAngle);
            return this;
        }
    }
}

/* Visualizer: Run snippet then dump into Desmos 3D
        ShotTable2 table = new ShotTable2();
        for (Map.Entry<Double, ShotTable2.ShotPoint> entry : table.table.entrySet()) {
            ShotTable2.ShotPoint shotPoint = entry.getValue();
            for (Map.Entry<Double, Double> entry2 : shotPoint.launchTable.entrySet()) {
                System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3.0f", entry.getKey(), entry2.getKey(), Math.toDegrees(entry2.getValue())));
            }
        }
        System.out.println("\n\n");
        for (double d = 35; d <= 160; d += 5) {
            double tv = table.getFlywheelForDistance(d);
            System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3d", d, tv, 0));
            for (double v = 390; v <= 670; v += 10) {
                double h = table.getLaunchAngleForDistanceAndFlywheel(d, v);
                System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3.0f", d, v, Double.isNaN(h) ? 100.0 : Math.toDegrees(h)));
            }
        }
*/

