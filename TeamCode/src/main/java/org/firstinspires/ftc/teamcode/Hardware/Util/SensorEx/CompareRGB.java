package org.firstinspires.ftc.teamcode.Hardware.Util.SensorEx;

public class CompareRGB {
    private double accuracyTreshold;
    private double r, g, b;

    public CompareRGB(double accuracyThreshold, double r, double g, double b) {
        this.accuracyTreshold = accuracyThreshold;
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public boolean isSameColor(double currentR, double currentG, double currentB) {
        return (Math.abs(currentR - r) <= accuracyTreshold &&
                Math.abs(currentG - g) <= accuracyTreshold &&
                Math.abs(currentB - b) <= accuracyTreshold);
    }

    public double getR() { return r; }

    public double getG() { return g;}

    public double getB() { return b; }

    public double getAccuracyThreshold() { return accuracyTreshold; }

    public void setAccuracyThreshold(double newAccuracyThreshold) { accuracyTreshold = newAccuracyThreshold; }
}
