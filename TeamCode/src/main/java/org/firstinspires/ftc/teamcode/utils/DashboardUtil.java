package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double ROBOT_RADIUS = 9; // in

    public static void drawField() {
        final int[] mirror = {1, -1};
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setFill("#e0e0e0e0");
        canvas.fillRect(-72, -72, 144, 144); // background
        canvas.setStroke("#808080");
        canvas.strokeRect(-15, -24, 30, 48); // submersible
        for (int i : mirror) {
            for (int j : mirror) canvas.strokeLine(i * 24, j * 24, i * 36, 0); // ascent zone
            canvas.strokeLine(i * -24, i *  24, i *  24, i *  24);
        }
        canvas.setStroke("#0000ff");
        for (int i : mirror) {
            canvas.strokeLine(i *  48, i *  72, i *  72, i *  48); // net zone
            canvas.strokeLine(i * -72, i *  60, i * -48, i *  60); // observation zone
            canvas.strokeLine(i * -48, i *  60, i * -36, i *  72); // observation zone
            canvas.strokeLine(i * -15, i *  24, i *  15, i *  24); // specimen rung
            for (int j : mirror) {
                for (int k = 0; k < 3; ++k) {
                    canvas.strokeLine(i * j * (-49.5 - 10 * k), i * 24, i * j * (-49.5 - 10 * k), i * 27.5); // preset
                }
                canvas.setStroke("#ffff00");
            }
            canvas.setStroke("#ff0000");
        }
        canvas.drawGrid(0, 0, 144, 144, 7, 7);
    }

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, String color) { drawRobot(canvas, pose, color, 2); }
    public static void drawRobot(Canvas canvas, Pose2d pose, String color, int lineWidth) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(lineWidth);
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        canvas.strokeCircle(pose.getX(), pose.getY(), 0.5);
        Pose2d v = new Pose2d(Math.cos(pose.heading)*ROBOT_RADIUS, Math.sin(pose.heading)*ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, String color, double turretAngle1, String turretColor1, double turretAngle2, String turretColor2) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS);

        double rdx = Math.cos(pose.heading), rdy = Math.sin(pose.heading);
        double rx1 = pose.x + rdx * 6, ry1 = pose.y + rdy * 6;
        double rx2 = pose.x + rdx * ROBOT_RADIUS, ry2 = pose.y + rdy * ROBOT_RADIUS;
        canvas.strokeLine(rx1, ry1, rx2, ry2);

        canvas.setStroke(turretColor1);
        canvas.setStrokeWidth(5);
        double t1dx = Math.cos(pose.heading + turretAngle1), t1dy = Math.sin(pose.heading + turretAngle1);
        double t1x1 = pose.x + t1dx * 96, t1y1 = pose.y + t1dy * 96;
        canvas.strokeLine(pose.x, pose.y, t1x1, t1y1);

        canvas.setStroke(turretColor2);
        canvas.setStrokeWidth(1);
        double t2dx = Math.cos(pose.heading + turretAngle2), t2dy = Math.sin(pose.heading + turretAngle2);
        double t2x1 = pose.x + t2dx * ROBOT_RADIUS, t2y1 = pose.y + t2dy * ROBOT_RADIUS;
        double t2x2 = pose.x + t2dx * 96, t2y2 = pose.y + t2dy * 96;
        canvas.strokeLine(t2x1, t2y1, t2x2, t2y2);

        canvas.setStrokeWidth(2);
        canvas.setStroke(color);
        canvas.strokeCircle(pose.x, pose.y, 0.5);
    }
}