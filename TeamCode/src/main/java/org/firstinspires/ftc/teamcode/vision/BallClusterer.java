package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pile of POLLEN / NECTAR to drive to.
 *
 * Steps:
 *  1. Filter: drop opponent NECTAR (G408, we'd only have to spit it out), anything outside the
 *     field, and optionally all NECTAR.
 *  2. Cluster: two balls are in the same cluster if they're within `linkDist` of each other,
 *     chained (single-linkage / DBSCAN with minPts = 1). A line of touching balls is one cluster,
 *     which is what we want, since the intake sweeps through it in one pass.
 *  3. Score: sum of ball weights (POLLEN 1, own NECTAR `nectarWeight`) minus a small penalty
 *     per inch of travel, so "biggest" wins and distance only breaks near-ties.
 *  4. Hysteresis: once we've picked a cluster, a different one has to beat it by
 *     `switchMargin` before we change target. Stops the robot dithering between two piles
 *     that score about the same.
 *  5. Approach pose: stand `approachStandoff` inches short of the centroid on the robot->cluster
 *     line, facing the centroid, so the final stretch is a straight drive through the pile.
 *
 * Pure Java, unit-tested in BallClustererTest.
 */
public class BallClusterer {

    public enum Type { POLLEN, RED_NECTAR, BLUE_NECTAR }

    public static class Ball {
        public final Vector2 pos;
        public final Type type;

        public Ball(Vector2 pos, Type type) {
            this.pos = pos;
            this.type = type;
        }
    }

    public static class Cluster {
        public final List<Ball> balls = new ArrayList<>();
        public Vector2 centroid;
        public double weight;   // weighted ball count
        public double score;    // weight minus distance penalty
        public double radius;   // farthest ball from centroid
    }

    /** Where to drive: the cluster and a pose to approach it from. */
    public static class Target {
        public final Cluster cluster;
        public final Vector2 approach;
        public final double heading;   // radians, field frame: facing the centroid

        Target(Cluster cluster, Vector2 approach, double heading) {
            this.cluster = cluster;
            this.approach = approach;
            this.heading = heading;
        }
    }

    // ---- Parameters (BallDetection copies its @Config values in here every loop) ----
    public double linkDist = 7.0;          // in. ~2 NECTAR diameters: balls this close get swept together
    public double nectarWeight = 1.0;      // value of one own-alliance NECTAR relative to a POLLEN
    public boolean collectNectar = true;
    public double distancePenalty = 0.01;  // score per inch of travel. 0.01 = 100 in costs one ball
    public double switchMargin = 0.75;     // a new cluster must beat the current one by this much
    public double approachStandoff = 14.0; // in from centroid to the approach pose
    public double fieldHalfWidth = 72.0;
    public double wallMargin = 1.5;        // ignore "balls" closer than this to/behind the wall
    public boolean isRed = true;

    private Vector2 lastCentroid = null;

    public boolean allowed(Ball b) {
        if (Math.abs(b.pos.x) > fieldHalfWidth - wallMargin || Math.abs(b.pos.y) > fieldHalfWidth - wallMargin) return false;
        switch (b.type) {
            case POLLEN: return true;
            case RED_NECTAR: return collectNectar && isRed;
            case BLUE_NECTAR: return collectNectar && !isRed;
            default: return false;
        }
    }

    public double weightOf(Ball b) { return b.type == Type.POLLEN ? 1.0 : nectarWeight; }

    /** All clusters of allowed balls, scored from `from`, best first. */
    public List<Cluster> cluster(List<Ball> input, Vector2 from) {
        List<Ball> balls = new ArrayList<>();
        for (Ball b : input) if (allowed(b)) balls.add(b);

        int n = balls.size();
        int[] label = new int[n];
        java.util.Arrays.fill(label, -1);
        List<Cluster> clusters = new ArrayList<>();
        double link2 = linkDist * linkDist;

        // Flood fill: each unlabeled ball starts a cluster, grow it through every neighbor in range.
        for (int seed = 0; seed < n; seed++) {
            if (label[seed] != -1) continue;
            int id = clusters.size();
            Cluster c = new Cluster();
            clusters.add(c);
            ArrayList<Integer> stack = new ArrayList<>();
            stack.add(seed);
            label[seed] = id;
            while (!stack.isEmpty()) {
                int i = stack.remove(stack.size() - 1);
                c.balls.add(balls.get(i));
                for (int j = 0; j < n; j++) {
                    if (label[j] == -1 && Vector2.distanceSquared(balls.get(i).pos, balls.get(j).pos) <= link2) {
                        label[j] = id;
                        stack.add(j);
                    }
                }
            }
        }

        for (Cluster c : clusters) {
            double sx = 0, sy = 0, w = 0;
            for (Ball b : c.balls) {
                double bw = weightOf(b);
                sx += b.pos.x * bw;
                sy += b.pos.y * bw;
                w += bw;
            }
            c.weight = w;
            c.centroid = w > 0 ? new Vector2(sx / w, sy / w) : c.balls.get(0).pos;
            double r = 0;
            for (Ball b : c.balls) r = Math.max(r, Vector2.distance(b.pos, c.centroid));
            c.radius = r;
            c.score = w - distancePenalty * Vector2.distance(from, c.centroid);
        }

        clusters.sort((a, b) -> Double.compare(b.score, a.score));
        return clusters;
    }

    /** Best cluster with hysteresis against the previous pick, or null if nothing allowed is visible. */
    public Cluster best(List<Ball> balls, Vector2 from) {
        List<Cluster> clusters = cluster(balls, from);
        if (clusters.isEmpty()) {
            return null; // keep lastCentroid: the pile may just be out of view for a moment
        }
        Cluster top = clusters.get(0);

        if (lastCentroid != null) {
            // Find whichever cluster is the one we were already going to.
            Cluster current = null;
            double bestD = Math.max(linkDist, 6.0);
            for (Cluster c : clusters) {
                double d = Vector2.distance(c.centroid, lastCentroid);
                if (d < bestD) { bestD = d; current = c; }
            }
            if (current != null && current != top && top.score < current.score + switchMargin) {
                top = current;
            }
        }
        lastCentroid = top.centroid;
        return top;
    }

    /** Best cluster plus the pose to approach it from, or null. */
    public Target target(List<Ball> balls, Vector2 robotPos) {
        Cluster c = best(balls, robotPos);
        if (c == null) return null;
        Vector2 toCluster = c.centroid.minus(robotPos);
        double dist = toCluster.mag();
        double heading = dist > 1e-6 ? Math.atan2(toCluster.y, toCluster.x) : 0;
        // Closer than the standoff already: approach pose is where we are (just turn and drive in).
        double back = Math.min(approachStandoff, dist);
        Vector2 dir = dist > 1e-6 ? toCluster.div(dist) : new Vector2(1, 0);
        Vector2 approach = c.centroid.minus(dir.times(back));
        return new Target(c, approach, heading);
    }

    /** Forget the current pick (call after collecting a cluster). */
    public void reset() { lastCentroid = null; }
}
