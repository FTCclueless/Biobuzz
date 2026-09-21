package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

import org.firstinspires.ftc.teamcode.utils.Vector2;

public final class Waypoint {
    public final Vector2 pos;
    public final double left;
    public final double right;

    public Waypoint(Vector2 pos, double left, double right) {
        if (left < 0 || right < 0) {
            throw new IllegalArgumentException("Corridor half-widths must be >= 0");
        }
        this.pos = pos;
        this.left = left;
        this.right = right;
    }

    public Waypoint(Vector2 pos, double corridor) {
        this(pos, corridor, corridor);
    }

    public Waypoint(double x, double y, double corridor) {
        this(new Vector2(x, y), corridor, corridor);
    }

    public static Waypoint pinned(double x, double y) {
        return new Waypoint(new Vector2(x, y), 0, 0);
    }

    public boolean isPinned() { return left == 0 && right == 0; }

    @Override
    public String toString() {
        return String.format("Waypoint%s [-%.1f, +%.1f]", pos, right, left);
    }
}
