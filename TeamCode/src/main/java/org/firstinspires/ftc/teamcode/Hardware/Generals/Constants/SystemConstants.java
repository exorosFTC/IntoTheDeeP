package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class SystemConstants {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double manualLiftCoefficient = 1;
    public static final int outtakeMAX = 300;
    public static final int extendoMAX = 170;

    public static final double rotationMin = 0;
    public static final double rotationMax = 0.48;

    public static final double inIntakeThreshold = 15; //mm
    public static final double v4bSafeDropdown = 60; //ticks

    public static final double inOuttakeThreshold = 15; //mm

    public static final ColorEx red = new ColorEx(240, 180, 130);
    public static final ColorEx blue = new ColorEx(96, 168, 250);
    public static final ColorEx yellow = new ColorEx(0, 0, 0);

    public static boolean ENCODER_VALUE_REJECTION = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;
    public static boolean autoDropdown = true;

    public static boolean usingOpenCvCamera = true;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;
}
