package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureSensor", group = "tuning")
public class SensorTest extends LinearOpMode {
    private ColorSensor sensor;
    private List<String> names = Arrays.asList("pixel1", "pixel2", "ramp_sensor");
    private Telemetry dashboardTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(ColorSensor.class, names.get(2));
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {
            dashboardTelemetry.addData("Red: ", sensor.red());
            dashboardTelemetry.addData("Green: ", sensor.green());
            dashboardTelemetry.addData("Blue: ", sensor.blue());
            dashboardTelemetry.update();
        }

    }
}
