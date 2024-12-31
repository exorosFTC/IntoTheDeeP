package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class SystemConstants {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static Enums.OuttakeEnums.ArmAction outtakeScore = Enums.OuttakeEnums.ArmAction.SCORE_SAMPLES;
    public static Enums.Color detectionColor = Enums.Color.YELLOW;

    public static Enums.OpMode opModeType;

    public static double manualLiftCoefficient = 1;
    public static final int outtakeMAX = 1650;
    public static final int extendoMAX = 470;

    public static double extensionMultiplier = 0.8;

    public static int distance_sensorToClaw = 30;

    public static final double rotationMin = 0;
    public static final double rotationMax = 0.65;
    public static final double rotationRange = rotationMax - rotationMin;

    public static final double rotationMinDeg = 335;
    public static final double rotationMaxDeg = 135;
    public static final double rotationRangeDeg = (rotationMinDeg > rotationMaxDeg) ? rotationMaxDeg - rotationMinDeg + 360 : rotationMaxDeg - rotationMinDeg;

    public static final double inIntakeThreshold = 25.5; //mm
    public static final double v4bSafeDropdown = 20; //ticks

    public static final double outtakeTicksPerDegree = 537.7 / 360;

    public static final ColorEx red = new ColorEx(530, 739, 626);
    public static final ColorEx blue = new ColorEx(404, 716, 719);
    public static final ColorEx yellow = new ColorEx(599, 981, 649);

    public static boolean ENCODER_VALUE_REJECTION = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static boolean usingOpenCvCamera = false;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;



}
