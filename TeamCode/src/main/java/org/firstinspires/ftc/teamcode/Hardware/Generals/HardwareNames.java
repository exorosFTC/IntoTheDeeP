package org.firstinspires.ftc.teamcode.Hardware.Generals;

import java.util.Arrays;
import java.util.List;


public class HardwareNames {
    public static final String cameraConfigurationName = "ExoCamera";
    public static final String IMU_Name = "ExoIMU";



    // TODO: modify your configuration names
    public static final String LeftFront = "LF";
    public static final String LeftBack = "LB";
    public static final String RightFront = "RF";
    public static final String RightBack = "RB";

    // you can leave these as they are
    public static final List<String> MotorNamesList = Arrays.asList(
            LeftFront, LeftBack, RightFront, RightBack);



    // TODO: tune these
    // If you use 2 or no odometry at all, set the names to "" (empty string)
    public static final String LeftOdometry = LeftFront;
    public static final String RightOdometry = LeftBack;
    public static final String PerpendicularOdometry = RightFront;

    // you can leave these as they are
    public static final List<String> EncoderNamesList = Arrays.asList(
            LeftOdometry, RightOdometry, PerpendicularOdometry);



    // TODO: add additional hardware components in the lists below
    public static List<String> ServoNamesList = Arrays.asList("", "");
    public static List<String> CRServoNamesList = Arrays.asList("", "");

    public static List<String> DigitalNamesList = Arrays.asList("", "");
    public static List<String> AnalogNamesList = Arrays.asList("", "");

    public static List<String> RevDistanceNameList = Arrays.asList("", "");
    public static List<String> RevColorNameList = Arrays.asList("", "");
    public static List<String> RevTouchNameList = Arrays.asList("", "");


}
