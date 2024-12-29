package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;

public class MecanumConstants {
    public static boolean usingDriveSensitivity = false;
    public static boolean usingButtonSensitivity = false;
    public static boolean usingVelocityToggle = true;
    public static boolean usingAcceleration = true;
    public static boolean usingExponentialInput = true;
    public static boolean usingFieldCentric = false;

    public static final double fastDrive = 1;
    public static final double slowDrive = 0.75;

    public static double speed = fastDrive;

    public static final double fastTurn = 0.6;
    public static final double slowTurn = 0.37;

    public static double accelerationScalar = 0.03;
    public static final double accelerationLiftExtended = 0.03;
    public static final double accelerationLiftRetracted = 0.04;

    public static double driveSensitivity = fastTurn;

    public static final double LinearP = 0.03,
                        LinearD = 0.0000000004;
    public static final double AngularP = 0.2,
                        AngularD = 0.01;

    // ODOMETRY CONSTANTS
    public static final double ODOMETRY_TICKS_PER_REVOLUTION = 2000;
    public static final double ODOMETRY_WHEEL_RADIUS_CM = 3.2;

    public static final Point forward = new Point(-1.2, 0.1);       //cm
    public static final Point perpendicular = new Point(0.9, 0.4); //cm

    public static final double TRACK_LENGTH = 29;
    public static final double TRACK_WIDTH = 5; //23 og, but it's turning too fast

    public static final double ODOMETRY_LEFT_X_OFFSET_CM = 0;
    public static final double ODOMETRY_RIGHT_X_OFFSET_CM = 0;
    public static final double ODOMETRY_PERPENDICULAR_Y_OFFSET_CM = 0;

    public static final double ODOMETRY_X_MULTIPLIER = 0.2;
    public static final double ODOMETRY_Y_MULTIPLIER = 0.2;
}
