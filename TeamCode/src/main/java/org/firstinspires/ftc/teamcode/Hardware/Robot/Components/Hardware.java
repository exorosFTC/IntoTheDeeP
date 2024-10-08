package org.firstinspires.ftc.teamcode.Hardware.Robot.Components;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.AnalogNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.CRServoNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.DigitalNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.EncoderNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.MotorNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevColorNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevDistanceNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevTouchNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware.Init;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.Encoder;

import java.util.HashMap;
import java.util.Map;

public class Hardware {
    private static Hardware instance;



    public Map<String, ServoEx> servos = new HashMap<>();
    public Map<String , CRServo> CRservos = new HashMap<>();

    public Map<String, AnalogInput> analog = new HashMap<>();
    public Map<String, DigitalChannel> digital = new HashMap<>();



    public Map<String, RevColorSensorV3> color = new HashMap<>();
    public Map<String, Rev2mDistanceSensor> distance = new HashMap<>();
    public Map<String, RevTouchSensor> touch = new HashMap<>();



    public Map<String, DcMotorEx> motors = new HashMap<>();
    private Map<String, Encoder> encoders = new HashMap<>();



    public static Hardware getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Hardware(hardwareMap);
        }
        return instance;
    }



    public Hardware(HardwareMap hardwareMap) {
        // add all servos into a list
        for (String servoName : ServoNamesList)
            if (!servoName.isEmpty())
                servos.put(servoName, hardwareMap.get(ServoEx.class, servoName));

        for (String CRServoName: CRServoNamesList)
            if (!CRServoName.isEmpty())
                CRservos.put(CRServoName, hardwareMap.get(CRServo.class, CRServoName));

        // add all motors into a list
        for (String motorName : MotorNamesList)
            if (!motorName.isEmpty())
                motors.put(motorName, Init.initializeMotor(hardwareMap.get(DcMotorEx.class, motorName)));

        // add all encoders into a list, without creating additional instances
        for (String encoderName : EncoderNamesList)
            if (!(motors.get(encoderName) == null || encoderName.isEmpty()))
                encoders.put(encoderName, new Encoder(hardwareMap.get(DcMotorEx.class, encoderName)));
            else if (!encoderName.isEmpty())
                encoders.put(encoderName, new Encoder(motors.get(encoderName)));


        for (String analogName : AnalogNamesList)
            if (!analogName.isEmpty())
                analog.put(analogName, hardwareMap.get(AnalogInput.class, analogName));

        for (String digitalName : DigitalNamesList)
            if (!digitalName.isEmpty())
                digital.put(digitalName, hardwareMap.get(DigitalChannel.class, digitalName));

        for (String distanceName : RevDistanceNameList)
            if (!distanceName.isEmpty())
                distance.put(distanceName, hardwareMap.get(Rev2mDistanceSensor.class, distanceName));

        for (String colorName : RevColorNameList)
            if (!colorName.isEmpty())
                color.put(colorName, hardwareMap.get(RevColorSensorV3.class, colorName));

        for (String touchName : RevTouchNameList)
            if (!touchName.isEmpty())
                touch.put(touchName, hardwareMap.get(RevTouchSensor.class, touchName));
    }
}
