package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.MecanumKinematics;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.follow.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.FrictionEllipse;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.profile.VelocityProfile;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.junit.Test;

import java.util.Locale;

public class PathingSimTest {
    static final double V_FORWARD = 81.175, V_STRAFE = 66.8431;
    static final double A_FORWARD = 70, A_STRAFE = 52, STALL = 3.5, SAFETY = 0.85;
    static final double TRACK = 14, BASE = 13, WHEEL_MAX = 85;
    static final double PEDRO_LINEAR = 0.0788, PEDRO_QUADRATIC = 0.00191035;
    static final double DT = 0.02;

    static FrictionEllipse ellipse() {
        return new FrictionEllipse(V_FORWARD, V_STRAFE, A_FORWARD, A_STRAFE, WHEEL_MAX,
                (TRACK + BASE) / 2.0, STALL);
    }

    static MecanumKinematics kinematics() { return new MecanumKinematics(TRACK, BASE, WHEEL_MAX); }

    static PathBuilder builder() { return new PathBuilder().ellipse(ellipse()).safetyFactor(SAFETY); }

    static Trajectory straight() {
        return builder().start(0, 0, 0).end(72, 0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.TANGENT).build();
    }

    static Trajectory curve() {
        return builder().start(0, 0, 0).waypoint(30, 6, 3).waypoint(48, 30, 3).end(48, 60, 0)
                .headingCandidate(PathBuilder.HeadingChoice.TANGENT).build();
    }

    static Trajectory sCurve() {
        return builder().start(0, 0, 0).waypoint(24, 18, 3).waypoint(48, -18, 3).end(72, 0, 0)
                .headingCandidate(PathBuilder.HeadingChoice.TANGENT)
                .headingCandidate(PathBuilder.HeadingChoice.FIXED_AT_END).endHeading(0).build();
    }

    @Test public void profilesRespectTheLimits() {
        double vCap = V_FORWARD * SAFETY + 1e-6;
        double aCap = A_FORWARD * SAFETY * STALL + 1e-6;
        double latCap = Math.max(A_FORWARD, A_STRAFE) * SAFETY * 1.02;
        for (Trajectory t : new Trajectory[]{straight(), curve(), sCurve()}) {
            VelocityProfile p = t.profile();
            assertEquals(0, p.velocity(0), 1e-6);
            assertEquals(0, p.velocity(t.length()), 1e-6);
            for (double s = 0; s <= t.length(); s += 0.25) {
                double v = p.velocity(s);
                assertTrue("speed over cap at s=" + s, v <= vCap);
                assertTrue("accel over cap at s=" + s, Math.abs(p.plannedAccel(s)) <= aCap);
                double lat = v * v * Math.abs(t.path().curvature(s));
                assertTrue(String.format(Locale.US, "lateral accel %.1f over cap at s=%.1f", lat, s),
                        lat <= latCap);
            }
        }
    }

    @Test public void brakingComparedWithPedroMeasuredModel() {
        Trajectory t = straight();
        VelocityProfile p = t.profile();
        double peak = 0, peakS = 0;
        for (double s = 0; s <= t.length(); s += 0.1) {
            if (p.velocity(s) > peak) { peak = p.velocity(s); peakS = s; }
        }
        double ourBrake = t.length() - peakS;
        double pedroBrake = PEDRO_LINEAR * peak + PEDRO_QUADRATIC * peak * peak;
        System.out.printf(Locale.US,
                "72in straight: peak %.1f in/s, planned time %.2fs; our braking %.1f in, "
                        + "Pedro's measured stop from that speed %.1f in%n",
                peak, t.duration(), ourBrake, pedroBrake);
        assertTrue("we brake later than the robot can physically stop", ourBrake >= pedroBrake);
    }

    static final class Run {
        boolean finished;
        double time, maxContour, endError;
    }

    static Run follow(Trajectory t, double simStatic, double followerStatic) {
        MecanumKinematics k = kinematics();
        SimChassis sim = new SimChassis(k, ellipse(), t.poseAt(0));
        sim.staticPower = simStatic;
        PathFollower f = new PathFollower(t, k).contourGain(3.2).lagGain(0.9).headingGain(4.0)
                .accelLead(0.03).strafeGain(1.0 / sim.strafeEfficiency)
                .staticPower(followerStatic);
        f.reset();
        Run r = new Run();
        double limit = t.duration() * 2.0 + 1.5;
        while (r.time < limit && !f.isFinished()) {
            PathFollower.Command c = f.update(sim.truePose(), DT);
            r.maxContour = Math.max(r.maxContour, Math.abs(c.contourError));
            sim.step(c.powers, DT);
            r.time += DT;
        }
        r.finished = f.isFinished();
        Pose2d end = t.poseAt(t.length());
        Pose2d at = sim.truePose();
        r.endError = Math.hypot(end.x - at.x, end.y - at.y);
        return r;
    }

    @Test public void followsEveryShapeOnFrictionlessWheels() {
        String[] names = {"straight", "curve", "s-curve"};
        Trajectory[] ts = {straight(), curve(), sCurve()};
        for (int i = 0; i < ts.length; i++) {
            Run r = follow(ts[i], 0, 0);
            System.out.printf(Locale.US, "%-8s planned %.2fs actual %.2fs  max contour %.2f in  end %.2f in%n",
                    names[i], ts[i].duration(), r.time, r.maxContour, r.endError);
            assertTrue(names[i] + " never finished", r.finished);
            assertTrue(names[i] + " drifted off the path", r.maxContour < 2.0);
            assertTrue(names[i] + " took far longer than planned", r.time < ts[i].duration() * 1.6 + 0.5);
        }
    }

    @Test public void frictionScanWithAndWithoutStaticCompensation() {
        for (double fr : new double[]{0.05, 0.10, 0.15, 0.20, 0.25}) {
            Run off = follow(curve(), fr, 0);
            Run on = follow(curve(), fr, fr);
            System.out.printf(Locale.US,
                    "friction %.2f: no kS finished=%b end %.2f in (%.2fs) | kS finished=%b end %.2f in (%.2fs)%n",
                    fr, off.finished, off.endError, off.time, on.finished, on.endError, on.time);
            assertTrue("kS run failed at friction " + fr, on.finished);
        }
    }

    @Test public void staticCompensationLetsItFinish() {
        for (Trajectory t : new Trajectory[]{straight(), curve(), sCurve()}) {
            Run r = follow(t, 0.07, 0.07);
            assertTrue("still stalled with kS", r.finished);
            assertTrue(r.endError < 1.0);
        }
    }
}
