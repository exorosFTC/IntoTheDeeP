package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.OutputStream;

public class UltrasonicSensor {
    private AnalogInput sensor;
    private LinearOpMode opMode;

    private final double MAX_mm = 500;
    private final double MAX_voltage = 3.3;


    public UltrasonicSensor(LinearOpMode opMode, String pinName) {
        sensor = opMode.hardwareMap.get(AnalogInput.class, pinName);
        this.opMode = opMode;
    }

    public double getDistance(DistanceUnit unit) {
        switch (unit) {
            case MM: { return sensor.getVoltage() / MAX_voltage * MAX_mm; }
            case CM: { return sensor.getVoltage() / MAX_voltage * MAX_mm * 0.1; }
            case INCH: { return sensor.getVoltage() / MAX_voltage * MAX_mm / 25.4; }
        }
        return sensor.getVoltage();
    }

    public double getVoltage() { return sensor.getVoltage(); }


}
