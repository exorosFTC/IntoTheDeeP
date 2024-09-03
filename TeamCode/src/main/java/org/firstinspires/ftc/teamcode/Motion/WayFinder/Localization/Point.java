package org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization;

public class Point {
    public double x;
    public double y;

    //......................................................................

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {
        this(0, 0);
    }

    //......................................................................

    public Point sum(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Pose sum(Pose other) { return new Pose(x + other.x, y + other.y, other.heading); }

    public Point sum(double val) { return new Point(x + val, y + val); }

    //......................................................................

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    //......................................................................

    public Point divideBy(double div) { return new Point(x / div, y / div); }

    public Point multiplyBy(double val) { return new Point(x * val, y * val); }

    //......................................................................

    public double distanceTo(Point other) { return other.subtract(this).hypot(); }

    public double atan(){ return Math.atan2(x, y); }

    public double hypot() { return Math.hypot(x, y); }

    public static Point toPolar(double r, double a){ return new Point(Math.cos(a)*r, Math.sin(a)*r); }

    public Point rotate_polar(double amount){ return Point.toPolar(hypot(), atan()+amount); }

    //......................................................................

}
