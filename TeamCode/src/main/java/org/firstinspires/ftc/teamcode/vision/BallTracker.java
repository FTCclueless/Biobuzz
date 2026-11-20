package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Short-term memory of where balls are on the field.
 *
 * Single frames are noisy: detections flicker, the distance estimate jitters by an inch or two,
 * and the balls right in front of the intake drop out of the camera's view as the robot gets
 * close. The tracker fixes all three:
 *  - each detection is merged into the nearest existing track (same type, within MERGE_DIST),
 *    smoothing its position;
 *  - a track has to be seen MIN_HITS times before it counts (kills one-frame false positives);
 *  - a track that stops being seen is kept for TIMEOUT_MS, so the target doesn't vanish
 *    the moment the balls go under the camera.
 *
 * Pure Java: time is passed in, so it unit-tests without a robot.
 */
public class BallTracker {
    public static class Track {
        public Vector2 pos;
        public BallClusterer.Type type;
        public int hits;
        public long lastSeenMs;

        Track(Vector2 pos, BallClusterer.Type type, long now) {
            this.pos = pos;
            this.type = type;
            this.hits = 1;
            this.lastSeenMs = now;
        }
    }

    public double mergeDist = 3.0;   // in. Smaller than one POLLEN diameter (2.8) + noise.
    public double smoothing = 0.35;  // 0..1, weight of a new measurement
    public int minHits = 2;
    public long timeoutMs = 1500;

    private final List<Track> tracks = new ArrayList<>();

    /** Feed one camera frame's worth of field-frame detections. */
    public void update(List<BallClusterer.Ball> detections, long nowMs) {
        // Only match against tracks from earlier frames: two balls in one frame are two balls.
        int existing = tracks.size();
        boolean[] claimed = new boolean[existing];
        for (BallClusterer.Ball d : detections) {
            int best = -1;
            double bestD = mergeDist;
            for (int i = 0; i < existing; i++) {
                Track t = tracks.get(i);
                if (claimed[i] || t.type != d.type) continue;
                double dist = Vector2.distance(t.pos, d.pos);
                if (dist < bestD) { bestD = dist; best = i; }
            }
            if (best >= 0) {
                Track t = tracks.get(best);
                t.pos = t.pos.plus(d.pos.minus(t.pos).times(smoothing));
                t.hits++;
                t.lastSeenMs = nowMs;
                claimed[best] = true;
            } else {
                tracks.add(new Track(d.pos, d.type, nowMs));
            }
        }
        prune(nowMs);
    }

    /** Drop tracks that haven't been seen recently. Call even on frames with no detections. */
    public void prune(long nowMs) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            if (nowMs - it.next().lastSeenMs > timeoutMs) it.remove();
        }
    }

    /** Forget a region, e.g. the balls the robot just drove through and intook. */
    public void clearAround(Vector2 center, double radius) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            if (Vector2.distance(it.next().pos, center) <= radius) it.remove();
        }
    }

    public void clear() { tracks.clear(); }

    /** Confirmed balls only (seen at least minHits times). */
    public List<BallClusterer.Ball> confirmed() {
        List<BallClusterer.Ball> out = new ArrayList<>();
        for (Track t : tracks) {
            if (t.hits >= minHits) out.add(new BallClusterer.Ball(t.pos, t.type));
        }
        return out;
    }

    public List<Track> allTracks() { return tracks; }
}
