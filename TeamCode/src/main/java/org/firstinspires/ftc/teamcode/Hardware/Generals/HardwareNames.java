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

    public static final String IntakeMotor = "intakeMotor";
    public static final String LeftOuttakeMotor = "leftOuttakeMotor";
    public static final String RightOuttakeMotor = "rightOuttakeMotor";


    // you can leave these as they are
    public static final List<String> MotorNamesList = Arrays.asList(
            LeftFront, LeftBack, RightFront, RightBack,
            IntakeMotor, LeftOuttakeMotor, RightOuttakeMotor);



    // TODO: tune these
    // If you use 2 or no odometry at all, set the names to "" (empty string)
    public static final String LeftOdometry = LeftFront;
    public static final String RightOdometry = "";
    public static final String PerpendicularOdometry = RightBack;

    // you can leave these as they are
    public static final List<String> EncoderNamesList = Arrays.asList(
            LeftOdometry, PerpendicularOdometry
    );



    // TODO: add additional hardware components in the lists below
    public static final String IntakeV4B = "IntakeV4B";
    public static final String IntakeClaw = "intakeClaw";
    public static final String IntakeRotation = "intakeRotation";

    public static final String OuttakeLeftPivot = "outtakeLeft";
    public static final String OuttakeRightPivot = "outtakeRight";

    public static final String OuttakeExtension = "outtakeExtension";
    public static final String OuttakeWrist = "outtakeWrist";
    public static final String OuttakeClaw = "outtakeClaw";

    public static List<String> ServoNamesList = Arrays.asList(
            IntakeV4B, IntakeClaw, IntakeRotation,
            OuttakeLeftPivot, OuttakeRightPivot, OuttakeExtension, OuttakeWrist, OuttakeClaw
    );




    public static List<String> CRServoNamesList = Arrays.asList("", "");
    public static List<String> DigitalNamesList = Arrays.asList("", "");
    public static List<String> AnalogNamesList = Arrays.asList("", "");




    public static final String OuttakeDistance = "outtakeDistanceSensor";

    public static List<String> RevDistanceNameList = Arrays.asList(
            OuttakeDistance
    );




    public static final String IntakeColor = "intakeColorSensor";

    public static List<String> RevColorNameList = Arrays.asList(
            IntakeColor
    );


    public static final String IntakeExtensionTouch = "intakeTouch";

    public static List<String> RevTouchNameList = Arrays.asList(
            IntakeExtensionTouch
    );


}
