package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class SystemConstants {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static double manualLiftCoefficient = 0.1;
    public static final int outtakeMAX = 300;
    public static final int extendoMAX = 300;

    public static final double inTransferThreshold = 0;
    public static final double inIntakeTreshold = 0;
    public static final double overBarThreshold = 0;
    public static final double encoderMaxTicks = 0;

    public static final ColorEx red = new ColorEx(255, 0, 0);
    public static final ColorEx blue = new ColorEx(0, 0, 255);

    public static boolean ENCODER_VALUE_REJECTION = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;
    public static boolean autoDropdown = true;

    public static boolean usingOpenCvCamera = true;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;
}
