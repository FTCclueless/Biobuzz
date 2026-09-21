package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry;

import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.opt.MinCurvatureOptimizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.HeadingPlan;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.RegionConstraint;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.VelocityProfile;

public final class PathBuilder {
    public enum HeadingChoice {
        TANGENT,
        TANGENT_REVERSED,
        CONSTANT_START,
        LINEAR,
        FIXED_AT_END
    }

    private final List<Waypoint> waypoints = new ArrayList<Waypoint>();
    private final List<RegionConstraint> regions = new ArrayList<RegionConstraint>();
    private static final class PendingMarker {
        final double offset;
        final boolean fromEnd;
        final String name;
        final Runnable action;

        PendingMarker(double offset, boolean fromEnd, String name, Runnable action) {
            this.offset = offset;
            this.fromEnd = fromEnd;
            this.name = name;
            this.action = action;
        }
    }

    private final List<PendingMarker> markers = new ArrayList<PendingMarker>();
    private final List<HeadingChoice> headingCandidates = new ArrayList<HeadingChoice>();

    private double startHeading = 0;
    private double endHeading = 0;
    private boolean endHeadingSet = false;
    private double vStart = 0;
    private double vEnd = 0;
    private FrictionEllipse ellipse = FrictionEllipse.typicalFtc();
    private double safetyFactor = 0.85;
    private double resolution = 0.5;
    private int optimizerSamples = 41;
    private int optimizerIterations = 12;
    private boolean optimize = true;
    private double minTurnRadius = 1.0;

    public PathBuilder start(double x, double y, double heading) {
        waypoints.add(Waypoint.pinned(x, y));
        this.startHeading = heading;
        return this;
    }

    public PathBuilder waypoint(double x, double y, double corridor) {
        waypoints.add(new Waypoint(x, y, corridor));
        return this;
    }

    public PathBuilder waypoint(double x, double y, double left, double right) {
        waypoints.add(new Waypoint(new Vector2(x, y), left, right));
        return this;
    }

    public PathBuilder end(double x, double y, double corridor) {
        waypoints.add(new Waypoint(x, y, corridor));
        return this;
    }

    public PathBuilder end(double x, double y, double corridor, double heading) {
        waypoints.add(new Waypoint(x, y, corridor));
        this.endHeading = heading;
        this.endHeadingSet = true;
        return this;
    }

    public PathBuilder endHeading(double heading) {
        this.endHeading = heading;
        this.endHeadingSet = true;
        return this;
    }

    public PathBuilder velocities(double startVelocity, double endVelocity) {
        this.vStart = startVelocity;
        this.vEnd = endVelocity;
        return this;
    }

    public PathBuilder slowZone(double x, double y, double radius, double maxVelocity) {
        regions.add(new RegionConstraint(x, y, radius, maxVelocity));
        return this;
    }

    public PathBuilder ellipse(FrictionEllipse e) {
        this.ellipse = e;
        return this;
    }

    public PathBuilder safetyFactor(double f) {
        this.safetyFactor = f;
        return this;
    }

    private FrictionEllipse plannedEllipse() {
        return safetyFactor == 1.0 ? ellipse : ellipse.scaled(safetyFactor);
    }

    public PathBuilder resolution(double inches) {
        this.resolution = inches;
        return this;
    }

    public PathBuilder headingCandidate(HeadingChoice c) {
        headingCandidates.add(c);
        return this;
    }

    public PathBuilder marker(double s, String name, Runnable action) {
        markers.add(new PendingMarker(s, false, name, action));
        return this;
    }

    public PathBuilder markerBeforeEnd(double inchesBefore, String name, Runnable action) {
        markers.add(new PendingMarker(inchesBefore, true, name, action));
        return this;
    }

    public PathBuilder skipOptimization() {
        this.optimize = false;
        return this;
    }

    public PathBuilder minTurnRadius(double inches) {
        this.minTurnRadius = inches;
        return this;
    }

    public PathBuilder optimizerSamples(int n) {
        this.optimizerSamples = Math.max(5, n);
        return this;
    }

    public Path buildPath() {
        if (waypoints.size() < 2) {
            throw new IllegalStateException("A path needs at least a start and an end");
        }
        Vector2[] pts = new Vector2[waypoints.size()];
        for (int i = 0; i < waypoints.size(); i++) pts[i] = waypoints.get(i).pos;

        Path seed = SplineFitter.fit(pts).toPath();
        if (!optimize || waypoints.size() < 3) {
            validate(seed);
            return seed;
        }

        int n = optimizerSamples;
        Vector2[] samples = SplineFitter.resample(seed, n);
        SplineFitter.Spline sp = SplineFitter.fit(samples);
        Vector2[] normals = new Vector2[n];
        for (int i = 0; i < n; i++) {
            normals[i] = sp.knotDeriv(i).unit().leftNormal();
        }

        double[] lo = new double[n];
        double[] hi = new double[n];
        double[] wpS = new double[waypoints.size()];
        double acc = 0;
        for (int i = 0; i < waypoints.size(); i++) {
            wpS[i] = seed.project(waypoints.get(i).pos, acc, seed.length());
            acc = wpS[i];
        }
        for (int i = 0; i < n; i++) {
            double s = seed.length() * i / (n - 1.0);
            double left = interpCorridor(wpS, s, true);
            double right = interpCorridor(wpS, s, false);
            lo[i] = -right;
            hi[i] = left;
        }
        lo[0] = hi[0] = 0;
        lo[n - 1] = hi[n - 1] = 0;

        MinCurvatureOptimizer.Result r = new MinCurvatureOptimizer()
                .maxIterations(optimizerIterations)
                .optimize(samples, normals, lo, hi);
        Path out = r.toPath();
        validate(out);
        return out;
    }

    private void validate(Path p) {
        if (minTurnRadius <= 0) return;
        double peak = p.peakCurvature();
        if (peak > 1.0 / minTurnRadius) {
            throw new IllegalStateException(String.format(
                    "Path has a turn radius of %.3f in (curvature %.2f 1/in), which is below "
                            + "the %.1f in minimum -- the spline almost certainly formed a cusp. "
                            + "Move or re-space the waypoints: this usually means one segment is "
                            + "much shorter than its neighbours. Call minTurnRadius(0) to bypass.",
                    1.0 / peak, peak, minTurnRadius));
        }
    }

    private double interpCorridor(double[] wpS, double s, boolean left) {
        int i = 0;
        while (i < wpS.length - 2 && wpS[i + 1] < s) i++;
        double s0 = wpS[i], s1 = wpS[i + 1];
        double c0 = left ? waypoints.get(i).left : waypoints.get(i).right;
        double c1 = left ? waypoints.get(i + 1).left : waypoints.get(i + 1).right;
        if (s1 - s0 < 1e-9) return Math.min(c0, c1);
        double f = Utils.minMaxClip((s - s0) / (s1 - s0), 0, 1);
        return c0 + (c1 - c0) * f;
    }

    public Trajectory build() {
        Path path = buildPath();
        if (headingCandidates.isEmpty()) {
            headingCandidates.add(HeadingChoice.TANGENT);
        }

        Trajectory best = null;
        double bestTime = Double.MAX_VALUE;
        for (HeadingChoice choice : headingCandidates) {
            HeadingPlan plan = makePlan(choice, path);
            VelocityProfile profile = VelocityProfile.builder(path)
                    .ellipse(plannedEllipse())
                    .heading(plan)
                    .startVelocity(vStart)
                    .endVelocity(vEnd)
                    .resolution(resolution)
                    .regions(regions)
                    .build();
            if (profile.duration() < bestTime) {
                bestTime = profile.duration();
                best = new Trajectory(path, plan, profile, resolveMarkers(path.length()));
            }
        }
        return best;
    }

    public String compareHeadings() {
        Path path = buildPath();
        StringBuilder sb = new StringBuilder();
        List<HeadingChoice> cands = headingCandidates.isEmpty()
                ? java.util.Arrays.asList(HeadingChoice.values()) : headingCandidates;
        for (HeadingChoice choice : cands) {
            VelocityProfile p = VelocityProfile.builder(path)
                    .ellipse(plannedEllipse()).heading(makePlan(choice, path))
                    .startVelocity(vStart).endVelocity(vEnd)
                    .resolution(resolution).regions(regions).build();
            sb.append(String.format("   %-18s %.3f s%n", choice, p.duration()));
        }
        return sb.toString();
    }

    private HeadingPlan makePlan(HeadingChoice choice, Path path) {
        double end = endHeadingSet ? endHeading : path.tangent(path.length()).theta();
        switch (choice) {
            case TANGENT:
                return new HeadingPlan.Tangent(path);
            case TANGENT_REVERSED:
                return new HeadingPlan.Tangent(path, Math.PI);
            case CONSTANT_START:
                return new HeadingPlan.Constant(startHeading);
            case LINEAR:
                return new HeadingPlan.Linear(startHeading, end, path.length());
            case FIXED_AT_END:
            default:
                double blend = Math.min(18.0, path.length() * 0.4);
                return new HeadingPlan.TangentThenFixed(
                        path, path.length() - blend, blend, end);
        }
    }

    private List<Trajectory.Marker> resolveMarkers(double length) {
        List<Trajectory.Marker> out = new ArrayList<Trajectory.Marker>(markers.size());
        for (PendingMarker m : markers) {
            double s = m.fromEnd ? length - m.offset : m.offset;
            out.add(new Trajectory.Marker(Utils.minMaxClip(s, 0, length), m.name, m.action));
        }
        return out;
    }
}
