package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

public class MecanumConstants {
    public static boolean usingDriveSensitivity = false;
    public static boolean usingButtonSensitivity = false;
    public static boolean usingHeadingCorrection = false;
    public static boolean usingVelocityToggle = true;

    public static final double fastConstrainScalar = 1;
    public static final double slowConstrainScalar = 0.4;

    // ODOMETRY CONSTANTS
    public static final double ODOMETRY_TICKS_PER_REVOLUTION = 8192;
    public static final double ODOMETRY_WHEEL_RADIUS_CM = 4;
    public static final double ODOMETRY_GEAR_RATIO = 1; // output (dead wheel) speed / input (encoder) speed

    public static final double TRACK_LENGTH = 0;
    public static final double TRACK_WIDTH = 0;
}
