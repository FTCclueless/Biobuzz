package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;

public interface HeadingPlan {
    double heading(double s);

    double dHeadingDs(double s);

    default double heading(double s, Path.Sample sm) { return heading(s); }

    default double dHeadingDs(double s, Path.Sample sm) { return dHeadingDs(s); }

    final class Tangent implements HeadingPlan {
        private final Path path;
        private final double offset;

        public Tangent(Path path) { this(path, 0); }

        public Tangent(Path path, double offset) {
            this.path = path;
            this.offset = offset;
        }

        @Override
        public double heading(double s) {
            return Utils.headingClip(path.tangent(s).theta() + offset);
        }

        @Override
        public double dHeadingDs(double s) { return path.curvature(s); }

        @Override
        public double heading(double s, Path.Sample sm) {
            return Utils.headingClip(Math.atan2(sm.ty, sm.tx) + offset);
        }

        @Override
        public double dHeadingDs(double s, Path.Sample sm) { return sm.curvature; }
    }

    final class Constant implements HeadingPlan {
        private final double h;

        public Constant(double heading) { this.h = Utils.headingClip(heading); }

        @Override
        public double heading(double s) { return h; }

        @Override
        public double dHeadingDs(double s) { return 0; }
    }

    final class Linear implements HeadingPlan {
        private final double start;
        private final double delta;
        private final double length;

        public Linear(double startHeading, double endHeading, double pathLength) {
            this.start = startHeading;
            this.delta = Utils.headingClip(endHeading - startHeading);
            this.length = Math.max(pathLength, 1e-6);
        }

        @Override
        public double heading(double s) {
            return Utils.headingClip(start + delta * Utils.minMaxClip(s / length, 0, 1));
        }

        @Override
        public double dHeadingDs(double s) {
            return delta / length;
        }
    }

    final class TangentThenFixed implements HeadingPlan {
        private final Path path;
        private final double sSwitch;
        private final double blend;
        private final double fixed;
        private final double pathLength;
        private final MathUtil.Curve headingCurve = new MathUtil.Curve() {
            @Override public double at(double x) { return heading(x); }
        };

        public TangentThenFixed(Path path, double sSwitch, double blendLength, double fixedHeading) {
            this.path = path;
            this.sSwitch = sSwitch;
            this.blend = Math.max(blendLength, 1e-6);
            this.fixed = Utils.headingClip(fixedHeading);
            this.pathLength = path.length();
        }

        private double blendFraction(double s) {
            return Utils.minMaxClip((s - sSwitch) / blend, 0, 1);
        }

        @Override
        public double heading(double s) {
            double u = blendFraction(s);
            if (u <= 0) return Utils.headingClip(path.tangent(s).theta());
            double tan = path.tangent(s).theta();
            double w = u * u * (3 - 2 * u);
            return Utils.headingClip(tan + Utils.headingClip(fixed - tan) * w);
        }

        @Override
        public double heading(double s, Path.Sample sm) {
            double u = blendFraction(s);
            double tan = Math.atan2(sm.ty, sm.tx);
            if (u <= 0) return Utils.headingClip(tan);
            double w = u * u * (3 - 2 * u);
            return Utils.headingClip(tan + Utils.headingClip(fixed - tan) * w);
        }

        @Override
        public double dHeadingDs(double s) {
            return MathUtil.angleDerivative(headingCurve, s, 0.25, 0, pathLength);
        }
    }
}
