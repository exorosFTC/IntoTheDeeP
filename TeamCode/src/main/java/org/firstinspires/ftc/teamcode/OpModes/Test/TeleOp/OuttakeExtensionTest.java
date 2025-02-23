package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;

@TeleOp(group = "test")
public class OuttakeExtensionTest extends LinearOpMode {
    private Hardware hardware;
    private GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        g2 = new GamepadEx(gamepad1);

        hardware = Hardware.getInstance(this);

        waitForStart();

        while (opModeIsActive()) {
            hardware.motors.get(LeftOuttakeMotor).setPower(g2.getLeftY());
            hardware.motors.get(RightOuttakeMotor).setPower(-g2.getLeftY());

            g2.readButtons();
            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }
    }
}
