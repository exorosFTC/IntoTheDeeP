package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "tuning", name = "SensorCalibration")
public class SensorRatioCalibration extends LinearOpMode {
    private RevColorSensorV3 pixel1, pixel2;
    private double rawR, rawG, rawB;

    private boolean isOnFirstSensor = true;

    private GamepadEx g2;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        pixel1 = hardwareMap.get(RevColorSensorV3.class, "pixel1");
        pixel2 = hardwareMap.get(RevColorSensorV3.class, "pixel2");

        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                isOnFirstSensor = !isOnFirstSensor;
            }

            if (isOnFirstSensor) {
                updateRatios(pixel1);
                dashboardTelemetry.addLine("pixel1");
            }
                else {
                    updateRatios(pixel2);
                    dashboardTelemetry.addLine("pixel2");
            }

            dashboardTelemetry.addData("R: ", rawR);
            dashboardTelemetry.addData("G: ", rawG);
            dashboardTelemetry.addData("B: ", rawB);
            dashboardTelemetry.update();
        }

    }

    public void updateRatios(RevColorSensorV3 sensor) {
        double d = getDistance(sensor);
        rawR = d / r(sensor);
        rawG = d / g(sensor);
        rawB = d / b(sensor);
    }

    private double getDistance(RevColorSensorV3 sensor) { return sensor.getDistance(DistanceUnit.CM); }

    private double r(RevColorSensorV3 sensor) { return sensor.red(); }
    private double g(RevColorSensorV3 sensor) { return sensor.green(); }
    private double b(RevColorSensorV3 sensor) { return sensor.blue(); }

}
