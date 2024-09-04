package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Pathing.RR.util.Encoder;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureEncoder", group = "tuning")
public class EncoderTest extends LinearOpMode {
    private Encoder encoder;
    private List<String> names = Arrays.asList("FL", "FR", "BL", "BR");
    private Telemetry dashboardTelemetry;

    private int i = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, names.get(i)));
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            dashboardTelemetry.addData("position: ", encoder.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }
}
