package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

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
        filteredDerivative = 0;
        counter = 0;
    }
    public double getIntegral() { return integral; }
    public void clipIntegral(double min, double max) {
        integral = Utils.minMaxClip(integral, min, max);
    }

    public void clipIntegralOutput(double min, double max) {
        if (Math.abs(i) < 1e-12) return;
        double a = min / i, b = max / i;
        integral = Utils.minMaxClip(integral, Math.min(a, b), Math.max(a, b));
    }

    public static double derivativeAlpha = 0.3;

    private double filteredDerivative = 0;

    public double update(double error, double min, double max) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1.0e9;
        lastLoopTime = currentTime;

        if (loopTime <= 1.0e-9) {
            return Utils.minMaxClip(p * error + i * integral + filteredDerivative, min, max);
        }

        double proportion = p * error;

        double integralTerm = i * (integral + error * loopTime);

        double rawDerivative = (error - lastError) / loopTime;
        if (counter == 0) {
            rawDerivative = 0;
            filteredDerivative = 0;
        }
        filteredDerivative += (d * rawDerivative - filteredDerivative) * derivativeAlpha;

        double unclipped = proportion + integralTerm + filteredDerivative;
        double output = Utils.minMaxClip(unclipped, min, max);

        boolean saturated = unclipped != output;
        boolean pushingFurther = (unclipped > max && error > 0) || (unclipped < min && error < 0);
        if (!(saturated && pushingFurther)) {
            integral += error * loopTime;
        }

        lastError = error;
        counter ++;

        return output;
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
