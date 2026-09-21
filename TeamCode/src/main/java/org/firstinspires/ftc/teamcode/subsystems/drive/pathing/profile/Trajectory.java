package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;

public final class Trajectory {
    public static final class Marker {
        public final double s;
        public final String name;
        public final Runnable action;

        public Marker(double s, String name, Runnable action) {
            this.s = s;
            this.name = name;
            this.action = action;
        }
    }

    private final Path path;
    private final HeadingPlan heading;
    private final VelocityProfile profile;
    private final List<Marker> markers;
    private int nextMarker = 0;

    public Trajectory(Path path, HeadingPlan heading, VelocityProfile profile, List<Marker> markers) {
        this.path = path;
        this.heading = heading;
        this.profile = profile;
        this.markers = new ArrayList<Marker>(markers);
        Collections.sort(this.markers, new Comparator<Marker>() {
            @Override
            public int compare(Marker a, Marker b) { return Double.compare(a.s, b.s); }
        });
    }

    public Path path() { return path; }

    public HeadingPlan heading() { return heading; }

    public VelocityProfile profile() { return profile; }

    public double length() { return path.length(); }

    public double duration() { return profile.duration(); }

    public List<Marker> markers() { return Collections.unmodifiableList(markers); }

    public Pose2d poseAt(double s) {
        return new Pose2d(path.point(s), heading.heading(s));
    }

    public double angularVelocityAt(double s) {
        return profile.velocity(s) * heading.dHeadingDs(s);
    }

    public double angularVelocityAt(double s, org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path.Sample sm) {
        return profile.velocity(s) * heading.dHeadingDs(s, sm);
    }

    public void fireMarkers(double s) {
        while (nextMarker < markers.size() && markers.get(nextMarker).s <= s) {
            Marker m = markers.get(nextMarker++);
            if (m.action != null) m.action.run();
        }
    }

    public List<String> pollMarkers(double s) {
        List<String> fired = new ArrayList<String>();
        while (nextMarker < markers.size() && markers.get(nextMarker).s <= s) {
            fired.add(markers.get(nextMarker++).name);
        }
        return fired;
    }

    public void resetMarkers() { nextMarker = 0; }

    public String describe() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Trajectory %.1f in, %.2f s, vmax %.1f in/s%n",
                length(), duration(), profile.maxVelocity()));
        for (Marker m : markers) {
            sb.append(String.format("   marker '%s' at s=%.1f in (t~%.2f s)%n",
                    m.name, m.s, profile.time(m.s)));
        }
        return sb.toString();
    }

    public double headingError(double s, double measuredHeading) {
        return Utils.headingClip(heading.heading(s) - measuredHeading);
    }
}
