package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;

import java.util.Arrays;
import java.util.List;

@TeleOp(group = "test", name = "sample")
public class SampleVisionTest extends LinearOpMode {
    private Camera camera;
    private Telemetry dashboardTelemetry;
    private GamepadEx g1;

    private int i = 0;
    private final List<Enums.Color> colors = Arrays.asList(Enums.Color.YELLOW, Enums.Color.RED, Enums.Color.BLUE);


    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new Camera(this, dashboardTelemetry);
        g1 = new GamepadEx(gamepad1);

        camera.setPipeline(Enums.Pipelines.DETECTING_SAMPLE);
        SystemConstants.detectionColor = colors.get(i);

        waitForStart();

        while (opModeIsActive()) {
            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                i += 1;
                if (i > 2) i = 0;

                SystemConstants.detectionColor = colors.get(i);
            }

            dashboardTelemetry.addData("Target Color: ", SystemConstants.detectionColor);
            dashboardTelemetry.addData("Randomization: ", camera.getRandomization());
            dashboardTelemetry.update();
            g1.readButtons();
        }
    }
}
