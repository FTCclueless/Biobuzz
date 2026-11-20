package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

public class Utils {
    public Utils() {}

    public static double minMaxClip(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public static int minMaxClip(int value, int min, int max) {
        return Math.min(Math.max(min, value), max);
    }

    public static double headingClip(double value) {
        double a = value % (2*Math.PI);
        if (a > Math.PI) a -= 2*Math.PI;
        else if (a <= -Math.PI) a += 2*Math.PI;
        return a;
    }

    public static boolean withinThreshold(double value, double minThreshold, double maxThreshold) {
        return value > minThreshold && value < maxThreshold;
    }

    public static double kalmanFilter (double value1, double value2, double value2Weight) {
        return (value1 * (1.0-value2Weight)) + (value2 * value2Weight);
    }

    public static double calculateDistanceBetweenPoints (Pose2d point1, Pose2d point2) {
        double deltaX = point1.x - point2.x;
        double deltaY = point1.y - point2.y;
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public static Pose fromPose2d(Pose2d p2d)
    {
        return new Pose(p2d.x, p2d.y, p2d.heading);
    }
}
