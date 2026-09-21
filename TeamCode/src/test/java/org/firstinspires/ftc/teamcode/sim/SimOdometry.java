package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Random;

public final class SimOdometry {
    public static final class Config {
        public double translationScale = 1.0;
        public double headingScale = 1.0;
        public double headingDriftRate = 0.0;
        public double positionNoise = 0.0;
        public double headingNoise = 0.0;
        public double quantization = 0.0;
        public int latencyLoops = 0;
        public double visionPeriod = 0.0;
        public double visionNoise = 0.75;
        public double visionHeadingNoise = Math.toRadians(2.0);
        public double visionTrust = 1.0;
        public double initialOffsetX = 0.0;
        public double initialOffsetY = 0.0;
        public double initialOffsetHeading = 0.0;

        public Config copy() {
            Config c = new Config();
            c.translationScale = translationScale;
            c.headingScale = headingScale;
            c.headingDriftRate = headingDriftRate;
            c.positionNoise = positionNoise;
            c.headingNoise = headingNoise;
            c.quantization = quantization;
            c.latencyLoops = latencyLoops;
            c.visionPeriod = visionPeriod;
            c.visionNoise = visionNoise;
            c.visionHeadingNoise = visionHeadingNoise;
            c.visionTrust = visionTrust;
            c.initialOffsetX = initialOffsetX;
            c.initialOffsetY = initialOffsetY;
            c.initialOffsetHeading = initialOffsetHeading;
            return c;
        }
    }

    private final Config cfg;
    private final Random rng;

    private double estX, estY, estHeading;
    private double lastTrueX, lastTrueY, lastTrueHeading;
    private final Deque<Pose2d> latencyBuffer = new ArrayDeque<>();
    private double timeSinceVision = 0;
    private double elapsed = 0;

    private double worstError = 0;
    private double sumError = 0;
    private int samples = 0;
    private int visionFixes = 0;
    private double largestJump = 0;

    public SimOdometry(Pose2d truth, Config cfg, Random rng) {
        this.cfg = cfg;
        this.rng = rng;
        this.lastTrueX = truth.x;
        this.lastTrueY = truth.y;
        this.lastTrueHeading = truth.heading;
        this.estX = truth.x + cfg.initialOffsetX;
        this.estY = truth.y + cfg.initialOffsetY;
        this.estHeading = Utils.headingClip(truth.heading + cfg.initialOffsetHeading);
    }

    private double quantize(double v) {
        if (cfg.quantization <= 0) return v;
        return Math.round(v / cfg.quantization) * cfg.quantization;
    }

    public void update(Pose2d truth, double dt) {
        elapsed += dt;

        double dx = truth.x - lastTrueX;
        double dy = truth.y - lastTrueY;
        double dh = Utils.headingClip(truth.heading - lastTrueHeading);
        lastTrueX = truth.x;
        lastTrueY = truth.y;
        lastTrueHeading = truth.heading;

        dx *= cfg.translationScale;
        dy *= cfg.translationScale;
        dh = dh * cfg.headingScale + cfg.headingDriftRate * dt;

        if (cfg.positionNoise > 0) {
            dx += rng.nextGaussian() * cfg.positionNoise;
            dy += rng.nextGaussian() * cfg.positionNoise;
        }
        if (cfg.headingNoise > 0) {
            dh += rng.nextGaussian() * cfg.headingNoise;
        }

        dx = quantize(dx);
        dy = quantize(dy);

        estX += dx;
        estY += dy;
        estHeading = Utils.headingClip(estHeading + dh);

        timeSinceVision += dt;
        if (cfg.visionPeriod > 0 && timeSinceVision >= cfg.visionPeriod) {
            timeSinceVision = 0;
            visionFixes++;
            double obsX = truth.x + rng.nextGaussian() * cfg.visionNoise;
            double obsY = truth.y + rng.nextGaussian() * cfg.visionNoise;
            double obsH = truth.heading + rng.nextGaussian() * cfg.visionHeadingNoise;

            double beforeX = estX, beforeY = estY;
            estX += (obsX - estX) * cfg.visionTrust;
            estY += (obsY - estY) * cfg.visionTrust;
            estHeading = Utils.headingClip(
                    estHeading + Utils.headingClip(obsH - estHeading) * cfg.visionTrust);
            largestJump = Math.max(largestJump, Math.hypot(estX - beforeX, estY - beforeY));
        }

        double err = Math.hypot(estX - truth.x, estY - truth.y);
        worstError = Math.max(worstError, err);
        sumError += err;
        samples++;

        latencyBuffer.addLast(new Pose2d(estX, estY, estHeading));
        while (latencyBuffer.size() > cfg.latencyLoops + 1) latencyBuffer.removeFirst();
    }

    public Pose2d reported() {
        if (latencyBuffer.isEmpty()) return new Pose2d(estX, estY, estHeading);
        return latencyBuffer.peekFirst();
    }

    public double error() { return worstError; }

    public double meanError() { return samples == 0 ? 0 : sumError / samples; }

    public int visionFixes() { return visionFixes; }

    public double largestVisionJump() { return largestJump; }
}
