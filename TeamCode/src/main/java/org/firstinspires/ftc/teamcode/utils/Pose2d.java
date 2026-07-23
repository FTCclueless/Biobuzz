package org.firstinspires.ftc.teamcode.utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Pose2d implements Cloneable {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y){
        this(x,y,0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void add(Pose2d p1) {
        this.x += p1.x;
        this.y += p1.y;
        this.heading += p1.heading;
    }

    public void subtract(Pose2d p1) {
        this.x -= p1.x;
        this.y -= p1.y;
        this.heading -= p1.heading;
    }

    public void mult(double k) {
        x *= k;
        y *= k;
        heading *= k;
    }

    public static Pose2d add(Pose2d... poses) {
        Pose2d res = new Pose2d(0, 0, 0);
        for(Pose2d pose : poses) {
            res.add(pose);
        }
        return res;
    }

    public boolean isNaN(){
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    public double getX(){ return x; }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }

    public double getDistanceFromPoint(Pose2d newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    public double getErrorInX(Pose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }

    public double getErrorInY(Pose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    public void clipAngle() {
        heading = AngleUtil.clipAngle(heading);
    }

    public double mag() { return Math.sqrt(x * x + y * y); }

    public static Pose2d from3D(Pose3D p) {
        return new Pose2d(p.getPosition().x, p.getPosition().y, p.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    public Pose2d mirror() {
        return new Pose2d(x, -y, -heading);
    }

    @NonNull
    @Override
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f)", x, y, heading);
    }
}