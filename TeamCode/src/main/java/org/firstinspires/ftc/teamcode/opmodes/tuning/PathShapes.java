package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public final class PathShapes {
    public static final int STRAIGHT = 0;
    public static final int L_SHAPE = 1;
    public static final int OUT_AND_BACK = 2;

    private PathShapes() {}

    public static Trajectory build(int shape, double length, FrictionEllipse ellipse, double safety) {
        PathBuilder b = new PathBuilder().ellipse(ellipse).safetyFactor(safety);

        switch (shape) {
            case L_SHAPE:
                return b.start(0, 0, 0)
                        .waypoint(length * 0.5, 0, 2.0)
                        .waypoint(length, length * 0.25, 2.0)
                        .end(length, length, 0)
                        .velocities(0, 0)
                        .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
                        .build();
            case OUT_AND_BACK:
                return b.start(0, 0, 0)
                        .waypoint(length, 0, 2.0)
                        .waypoint(length, 24, 2.0)
                        .end(0, 24, 0)
                        .velocities(0, 0)
                        .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
                        .build();
            default:
                return b.start(0, 0, 0)
                        .end(length, 0, 0)
                        .velocities(0, 0)
                        .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
                        .endHeading(0)
                        .build();
        }
    }

    public static Trajectory reverse(Trajectory forward, FrictionEllipse ellipse, double safety) {
        Pose2d from = forward.poseAt(forward.length());
        Pose2d to = forward.poseAt(0);
        return new PathBuilder()
                .ellipse(ellipse)
                .safetyFactor(safety)
                .start(from.x, from.y, from.heading)
                .end(to.x, to.y, 0, to.heading)
                .velocities(0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END)
                .endHeading(to.heading)
                .build();
    }

    public static String name(int shape) {
        switch (shape) {
            case L_SHAPE:      return "L";
            case OUT_AND_BACK: return "out and back";
            default:           return "straight";
        }
    }
}
