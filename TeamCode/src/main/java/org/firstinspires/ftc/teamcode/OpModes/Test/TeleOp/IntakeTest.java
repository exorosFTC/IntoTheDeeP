package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@TeleOp(name = "Intake", group = "Test")
public class IntakeTest extends ExoMode {
    private static Intake intake;
    private static GamepadEx g2;

    @Override
    protected void Init() {
        intake = new Intake(this);
        g2 = new GamepadEx(gamepad2);
    }

    @Override
    protected void WhenStarted() {}

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {
        if (g2.wasJustPressed(GamepadKeys.Button.B))
            intake.setAction(Enums.IntakeAction.COLLECT);

        if (g2.wasJustPressed(GamepadKeys.Button.A))
            intake.setAction(Enums.IntakeAction.TRANSFER);

        intake.extend(g2.getLeftY());

        g2.readButtons();

    }
}
