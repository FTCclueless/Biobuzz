package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.utils.Utils;

// https://www.ctrlaltftc.com/the-pid-controller
@Config
public class PID {
    public double p;
    public double i;
    public double d;
    public PID(double P, double I, double D) {
        p=P;
        i=I;
        d=D;
    }
    private double integral = 0;
    private long lastLoopTime = System.nanoTime();
    private double lastError = 0;
    private int counter = 0;
    private double loopTime = 0.0;

    public void resetIntegral() {
        integral = 0;
    }
    public double getIntegral() { return integral; }
    public void clipIntegral(double min, double max) {
        integral = Utils.clamp(integral, min, max);
    }

    public double update(double error, double min, double max) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1.0e9;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
        integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        counter ++;

        return Utils.clamp(proportion + integral + derivative, min, max);
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}