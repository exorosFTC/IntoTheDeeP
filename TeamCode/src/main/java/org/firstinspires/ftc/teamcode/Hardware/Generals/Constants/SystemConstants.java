package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class SystemConstants {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static Enums.OuttakeEnums.ArmAction outtakeScore = Enums.OuttakeEnums.ArmAction.SCORE_SAMPLES;

    public static double manualLiftCoefficient = 1;
    public static final int outtakeMAX = 1650;
    public static final int extendoMAX = 470;

    public static final double rotationMin = 0;
    public static final double rotationMax = 0.65;

    public static final double inIntakeThreshold = 30; //mm
    public static final double v4bSafeDropdown = 60; //ticks

    public static final double outtakeTicksPerDegree = 537.7 / 360;

    public static final ColorEx red = new ColorEx(240, 350, 277);
    public static final ColorEx blue = new ColorEx(195, 345, 310);
    public static final ColorEx yellow = new ColorEx(265, 420, 280);

    public static boolean ENCODER_VALUE_REJECTION = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static boolean usingOpenCvCamera = false;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;



}
