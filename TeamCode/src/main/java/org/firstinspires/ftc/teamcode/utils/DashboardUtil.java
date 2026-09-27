package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.geometry.Path;

import java.util.List;

public class DashboardUtil {
    private static final double ROBOT_RADIUS = 9;

    public static void drawField() {
        final int[] mirror = {1, -1};
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setFill("#e0e0e0e0");
        canvas.fillRect(-72, -72, 144, 144);
        canvas.setStroke("#808080");
        canvas.strokeRect(-15, -24, 30, 48);
        for (int i : mirror) {
            for (int j : mirror) canvas.strokeLine(i * 24, j * 24, i * 36, 0);
            canvas.strokeLine(i * -24, i *  24, i *  24, i *  24);
        }
        canvas.setStroke("#0000ff");
        for (int i : mirror) {
            canvas.strokeLine(i *  48, i *  72, i *  72, i *  48);
            canvas.strokeLine(i * -72, i *  60, i * -48, i *  60);
            canvas.strokeLine(i * -48, i *  60, i * -36, i *  72);
            canvas.strokeLine(i * -15, i *  24, i *  15, i *  24);
            for (int j : mirror) {
                for (int k = 0; k < 3; ++k) {
                    canvas.strokeLine(i * j * (-49.5 - 10 * k), i * 24, i * j * (-49.5 - 10 * k), i * 27.5);
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

    public static double[][] samplePath(Path path) {
        if (path == null) return null;
        int samples = Math.max(20, (int) (path.length() * 2));
        List<Vector2> pts = path.sampleUniform(samples);
        double[][] out = new double[2][pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            out[0][i] = pts.get(i).x;
            out[1][i] = pts.get(i).y;
        }
        return out;
    }

    public static void drawSampledPath(Canvas canvas, double[][] sampled) {
        if (sampled == null || sampled[0].length == 0) return;
        canvas.setStroke("#4CAF50");
        canvas.setStrokeWidth(1);
        canvas.strokePolyline(sampled[0], sampled[1]);
        int last = sampled[0].length - 1;
        canvas.strokeCircle(sampled[0][last], sampled[1][last], 2);
    }

    public static void drawPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, samplePath(path));
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

}
