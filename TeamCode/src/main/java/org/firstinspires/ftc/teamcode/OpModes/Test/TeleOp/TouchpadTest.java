package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;

@TeleOp(group = "test")
public class TouchpadTest extends LinearOpMode {
    private Intake intake;
    private Hardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(this);
        hardware = Hardware.getInstance(this);

        waitForStart();

        while (opModeIsActive()) {
            double raw = (gamepad2.touchpad_finger_1_x + 1) / 2;
            int position = (int) (raw * SystemConstants.extendoMAX);

            hardware.telemetry.addData("raw: ", raw);
            hardware.telemetry.addData("raw 2: ", gamepad2.touchpad_finger_1);
            hardware.telemetry.addData("position: ", position);
            hardware.telemetry.update();

            if (gamepad2.touchpad_finger_1)
                intake.extension.setPosition(position);

            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }
    }
}
