package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevColorNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureSensor", group = "tuning")
public class SensorTest extends LinearOpMode {
    private ColorRangeSensor sensor;
    private int index = 0;
    private Telemetry dashboardTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(RevColorSensorV3.class, RevColorNameList.get(index));
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {
            dashboardTelemetry.addData("Red: ", sensor.red());
            dashboardTelemetry.addData("Green: ", sensor.green());
            dashboardTelemetry.addData("Blue: ", sensor.blue());
            dashboardTelemetry.addData("Distance: ", sensor.getDistance(DistanceUnit.MM));
            dashboardTelemetry.update();
        }

    }
}
