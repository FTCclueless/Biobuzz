package org.firstinspires.ftc.teamcode.control;

public final class MotionProfile {
    private final double start;
    private final double goal;
    private final double direction;
    private final double maxVel;
    private final double accel;

    private final double tAccel;
    private final double tCruise;
    private final double tTotal;
    private final double vPeak;
    private final double dAccel;
    private final double dCruise;

    public MotionProfile(double start, double goal, double maxVelocity, double maxAccel) {
        if (maxVelocity <= 0 || maxAccel <= 0) {
            throw new IllegalArgumentException("maxVelocity and maxAccel must be > 0");
        }
        this.start = start;
        this.goal = goal;
        double distance = Math.abs(goal - start);
        this.direction = goal >= start ? 1 : -1;
        this.maxVel = maxVelocity;
        this.accel = maxAccel;

        double vTri = Math.sqrt(maxAccel * distance);
        if (vTri <= maxVelocity) {
            this.vPeak = vTri;
            this.tAccel = vTri / maxAccel;
            this.tCruise = 0;
            this.dAccel = distance / 2;
            this.dCruise = 0;
        } else {
            this.vPeak = maxVelocity;
            this.tAccel = maxVelocity / maxAccel;
            this.dAccel = maxVelocity * maxVelocity / (2 * maxAccel);
            this.dCruise = distance - 2 * dAccel;
            this.tCruise = dCruise / maxVelocity;
        }
        this.tTotal = 2 * tAccel + tCruise;
    }

    public static final class State {
        public final double position;
        public final double velocity;
        public final double acceleration;

        State(double p, double v, double a) {
            this.position = p;
            this.velocity = v;
            this.acceleration = a;
        }
    }

    public State get(double t) {
        if (tTotal <= 0) return new State(goal, 0, 0);
        if (t <= 0) return new State(start, 0, 0);
        if (t >= tTotal) return new State(goal, 0, 0);

        double p, v, a;
        if (t < tAccel) {
            a = accel;
            v = accel * t;
            p = 0.5 * accel * t * t;
        } else if (t < tAccel + tCruise) {
            double dt = t - tAccel;
            a = 0;
            v = vPeak;
            p = dAccel + vPeak * dt;
        } else {
            double dt = t - tAccel - tCruise;
            a = -accel;
            v = vPeak - accel * dt;
            p = dAccel + dCruise + vPeak * dt - 0.5 * accel * dt * dt;
        }
        return new State(start + direction * p, direction * v, direction * a);
    }

    public double duration() { return tTotal; }

    public double peakVelocity() { return vPeak; }

    public double goal() { return goal; }

    public boolean isFinished(double t) { return t >= tTotal; }

    @Override
    public String toString() {
        return String.format("MotionProfile[%.2f -> %.2f, %.3f s, vpeak %.2f]",
                start, goal, tTotal, vPeak);
    }
}
