package org.firstinspires.ftc.teamcode.Pathing.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Pose extends Point {
    public double heading;

    //......................................................................

    public Pose(double x, double y, double heading) {
        super(x,y);
        this.heading = heading;
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose() {
        this(0, 0, 0);
    }

    //......................................................................

    public Pose sum(Pose other) {
        return new Pose(this.x + other.x, this.y + other.y, AngleUnit.normalizeRadians(this.heading + other.heading));
    }

    //......................................................................

    public Pose subtract(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    //......................................................................

    public Pose multiplyBy(Pose other) {
        return new Pose(this.x * other.x, this.y * other.y, AngleUnit.normalizeRadians(this.heading * other.heading));
    }

    public Pose multiplyBy(double amount) {
        return new Pose(this.x * amount, this.y * amount, AngleUnit.normalizeRadians(this.heading * amount));
    }

    //......................................................................

    public Pose divideBy(Pose other) {
        return new Pose(this.x / other.x, this.y / other.y, AngleUnit.normalizeRadians(this.heading / other.heading)); }

    public Pose divideBy(double amount) {
        return  new Pose(this.x / amount, this.y / amount, AngleUnit.normalizeRadians(this.heading / amount)); }

    //......................................................................

    public Pose rotateWithRotationalMatrix(double amount) {
        double new_x = x * Math.cos(amount) - y * Math.sin(amount);
        double new_y = x * Math.sin(amount) + y * Math.cos(amount);

        return new Pose(new_x, new_y, heading);
    }

    public Point getPoint() { return new Point(x, y); }

    //......................................................................
}
