package org.firstinspires.ftc.teamcode.Pathing.MotionProfiling;

public class ProfileConstrains {
    public double MAX_VELOCITY;

    public double MAX_ACCELERATION;
    public double MAX_DECELERATION;

    public double MAX_JERK;
    public double MAX_DECEL_JERK;

    public ProfileConstrains(double v, double a, double d, double j, double i) {
        this.MAX_VELOCITY = v;
        this.MAX_ACCELERATION = a;
        this.MAX_DECELERATION = d;
        this.MAX_JERK = j;
        this.MAX_DECEL_JERK = i;
    }

    public ProfileConstrains(double v, double a, double d, double j) {
        this.MAX_VELOCITY = v;
        this.MAX_ACCELERATION = a;
        this.MAX_DECELERATION = d;
        this.MAX_JERK = j;
        this.MAX_DECEL_JERK = j;
    }

    public ProfileConstrains(double v, double a, double d) {
        this.MAX_VELOCITY = v;
        this.MAX_ACCELERATION = a;
        this.MAX_DECELERATION = d;
        this.MAX_JERK = 0;
        this.MAX_DECEL_JERK = 0;
    }

    public ProfileConstrains(double v, double a) {
        this.MAX_VELOCITY = v;
        this.MAX_ACCELERATION = a;
        this.MAX_DECELERATION = a;
        this.MAX_JERK = 0;
        this.MAX_DECEL_JERK = 0;
    }
}
