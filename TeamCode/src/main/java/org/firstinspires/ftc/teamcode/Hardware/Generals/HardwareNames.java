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
    public static final String RightOdometry = LeftBack;
    public static final String PerpendicularOdometry = RightFront;

    // you can leave these as they are
    public static final List<String> EncoderNamesList = Arrays.asList(
            LeftOdometry, RightOdometry, PerpendicularOdometry);



    // TODO: add additional hardware components in the lists below
    public static final String IntakeDropdown = "intakeDropdown";
    public static final String IntakeStopper = "intakeStopper";

    public static final String OuttakeLeftPivot = "outtakeLeft";
    public static final String OuttakeRightPivot = "outtakeRight";

    public static final String OuttakeExtension = "outtakeExtension";
    public static final String OuttakeWrist = "outtakeWrist";
    public static final String OuttakeClaw = "outtakeClaw";

    public static List<String> ServoNamesList = Arrays.asList(
            IntakeDropdown, IntakeStopper,
            OuttakeLeftPivot, OuttakeRightPivot, OuttakeExtension, OuttakeWrist, OuttakeClaw
    );



    public static final String IntakeLeftServo = "intakeLeft";
    public static final String IntakeRightServo = "intakeRight";

    public static List<String> CRServoNamesList = Arrays.asList(
            IntakeLeftServo, IntakeRightServo
    );

    public static List<String> DigitalNamesList = Arrays.asList("", "");
    public static List<String> AnalogNamesList = Arrays.asList("", "");



    public static final String IntakeDistance = "intakeDistanceSensor";
    public static final String TransferDistance = "transferDistanceSensor";

    public static List<String> RevDistanceNameList = Arrays.asList(
            IntakeDistance, TransferDistance
    );



    public static final String IntakeColor = "intakeColor";

    public static List<String> RevColorNameList = Arrays.asList(
            IntakeColor
    );


    public static final String IntakeExtensionTouch = "intakeTouch";

    public static List<String> RevTouchNameList = Arrays.asList(
            IntakeExtensionTouch
    );


}
