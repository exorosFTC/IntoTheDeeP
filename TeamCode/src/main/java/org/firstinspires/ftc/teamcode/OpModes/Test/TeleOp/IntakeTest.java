package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@TeleOp(name = "Intake", group = "Test")
public class IntakeTest extends ExoMode implements Enums.OuttakeEnums, Enums.IntakeEnums {
    private static Intake intake;
    private static Drivetrain drive;
    private static GamepadEx g1, g2;

    @Override
    protected void Init() {
        intake = new Intake(this);
        drive = new Drivetrain(this);

        g2 = new GamepadEx(gamepad2);
        g1 = new GamepadEx(gamepad1);

        InitializeThreads();
    }

    @Override
    protected void WhenStarted() {}

    @Override
    protected void InitializeThreads() {
        new Thread(() -> {
            waitForStart();

            while (opModeIsActive()) {
                drive.update(new Pose(
                        -g1.getLeftY(),
                        g1.getLeftX(),
                        -g1.getRightX() * 0.7
                ));

                g1.readButtons();
            }
        }).start();
    }


    @Override
    protected void Loop() {
        if (g2.wasJustPressed(GamepadKeys.Button.B))
            intake.setAction(IntakeAction.COLLECT);

        if (g2.wasJustPressed(GamepadKeys.Button.A))
            intake.setAction(IntakeAction.TRANSFER);

        if (g2.wasJustPressed(GamepadKeys.Button.Y))
            intake.toggleClaw();

        telemetry.addData("action: ", intake.getAction());
        telemetry.addData("current (Amps): ", intake.getCurrent());

        intake.rotate(g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                      g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        0.02);
        intake.extend(g2.getLeftY());

        g2.readButtons();

    }

}
