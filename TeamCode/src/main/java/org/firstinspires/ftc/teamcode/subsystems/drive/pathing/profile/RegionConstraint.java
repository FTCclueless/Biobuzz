package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile;

import org.firstinspires.ftc.teamcode.utils.Vector2;

public final class RegionConstraint {
    public final Vector2 center;
    public final double radius;
    public final double maxVelocity;

    public RegionConstraint(Vector2 center, double radius, double maxVelocity) {
        this.center = center;
        this.radius = radius;
        this.maxVelocity = maxVelocity;
    }

    public RegionConstraint(double x, double y, double radius, double maxVelocity) {
        this(new Vector2(x, y), radius, maxVelocity);
    }

    public boolean contains(Vector2 p) {
        return Vector2.distanceSquared(p, center) <= radius * radius;
    }

    public double cap(Vector2 p) {
        return contains(p) ? maxVelocity : Double.MAX_VALUE;
    }

    @Override
    public String toString() {
        return String.format("SlowZone%s r=%.1f v<=%.1f", center, radius, maxVelocity);
    }
}
