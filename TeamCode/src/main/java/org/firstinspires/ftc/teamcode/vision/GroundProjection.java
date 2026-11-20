package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utils.Vector2;

/**
 * Turns a camera angle measurement (Limelight tx/ty) into a point on the floor.
 *
 * Pure math, no hardware, so it runs in unit tests.
 *
 * Frames:
 *  - Robot frame: +x forward, +y left, +z up, origin at the robot's tracking center on the floor.
 *  - Field frame: same convention as ROBOT_POSITION (inches, radians, CCW positive).
 *
 * The ball's CENTER is what the detector box is centered on, so we intersect the camera ray
 * with the plane z = ballRadius, not z = 0. That matters: with a 10 in camera, a 1.8 in NECTAR
 * radius is an 18% height error if ignored.
 */
public final class GroundProjection {
    private GroundProjection() {}

    /**
     * @param txDeg        horizontal angle to target, degrees, positive = target to the RIGHT (Limelight convention)
     * @param tyDeg        vertical angle to target, degrees, positive = target ABOVE the crosshair
     * @param camForward   camera lens position forward of robot center (in)
     * @param camLeft      camera lens position left of robot center (in)
     * @param camHeight    camera lens height above the floor (in)
     * @param camPitchDown camera tilt toward the floor, degrees (positive = looking down)
     * @param camYaw       camera yaw relative to robot forward, degrees, CCW positive
     * @param targetHeight height of the point being looked at (ball radius) (in)
     * @param maxRange     reject anything farther than this from the camera (in); rays near the
     *                     horizon blow up, so this cuts off the garbage
     * @return the point in the ROBOT frame, or null if the ray never hits the plane / is too far
     */
    public static Vector2 toRobotFrame(double txDeg, double tyDeg,
                                       double camForward, double camLeft, double camHeight,
                                       double camPitchDown, double camYaw,
                                       double targetHeight, double maxRange) {
        double pitch = Math.toRadians(camPitchDown);
        double tanX = Math.tan(Math.toRadians(txDeg));
        double tanY = Math.tan(Math.toRadians(tyDeg));

        // Ray in camera-aligned axes before tilt: forward 1, left -tanX, up tanY.
        // Tilt it down by `pitch` about the camera's left axis.
        double fwd = Math.cos(pitch) + tanY * Math.sin(pitch);
        double up = -Math.sin(pitch) + tanY * Math.cos(pitch);
        double left = -tanX;

        double drop = camHeight - targetHeight;
        if (drop <= 0 || up >= -1e-6) return null; // ray points at or above the target plane

        double t = drop / -up;
        double dx = fwd * t, dy = left * t;
        if (Math.hypot(dx, dy) > maxRange) return null;

        // Yaw the camera-relative offset into the robot frame, then add the mount offset.
        double yaw = Math.toRadians(camYaw);
        double c = Math.cos(yaw), s = Math.sin(yaw);
        return new Vector2(camForward + dx * c - dy * s, camLeft + dx * s + dy * c);
    }

    /** Robot-frame point -> field-frame point, given the robot pose. */
    public static Vector2 robotToField(Vector2 robotPoint, double robotX, double robotY, double robotHeading) {
        double c = Math.cos(robotHeading), s = Math.sin(robotHeading);
        return new Vector2(robotX + robotPoint.x * c - robotPoint.y * s,
                robotY + robotPoint.x * s + robotPoint.y * c);
    }
}
