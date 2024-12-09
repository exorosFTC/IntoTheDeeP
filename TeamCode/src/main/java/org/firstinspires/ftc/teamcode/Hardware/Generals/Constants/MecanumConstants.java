package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

public class MecanumConstants {
    public static boolean usingDriveSensitivity = false;
    public static boolean usingButtonSensitivity = false;
    public static boolean usingVelocityToggle = true;
    public static boolean usingAcceleration = true;
    public static boolean usingExponentialInput = true;

    public static final double fastDrive = 1;
    public static final double slowDrive = 0.3;
    public static final double accelerationScalar = 0.03;

    public static double driveSensitivity = fastDrive;


    // ODOMETRY CONSTANTS
    public static final double ODOMETRY_TICKS_PER_REVOLUTION = 8192;
    public static final double ODOMETRY_WHEEL_RADIUS_CM = 4;

    public static final double TRACK_LENGTH = 29;
    public static final double TRACK_WIDTH = 5; //23 og, but it's turning too fast

    public static final double ODOMETRY_LEFT_X_OFFSET_CM = 0;
    public static final double ODOMETRY_RIGHT_X_OFFSET_CM = 0;
    public static final double ODOMETRY_PERPENDICULAR_Y_OFFSET_CM = 0;

    public static final double ODOMETRY_X_MULTIPLIER = 1;
    public static final double ODOMETRY_Y_MULTIPLIER = 1;
}
