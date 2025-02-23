package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;

import java.util.Arrays;

@TeleOp(group = "test")
public class intakeTest extends LinearOpMode {
    Intake intake;
    TriggerManager intakeTriggers;
    Hardware hardware;
    GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(this);
        hardware = Hardware.getInstance(this);
        g2 = new GamepadEx(gamepad2);

        intakeTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> {
                        intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);
                        intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_COLLECT);})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> {
                            intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                            intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_STOP);
                })

                .addTriggerSequence(() -> g2.wasJustPressed(GamepadKeys.Button.B),
                        Arrays.asList(
                                () -> intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_COLLECT),
                                () -> intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_STOP))
                )

                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.A),
                        () -> {
                            intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);
                            intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                })


                .addAction(() -> intake.rotate(-g2.getRightX(), 0.01))
                .addAction(() -> intake.extend(g2.getLeftY() * 0.9));


        waitForStart();


        while (opModeIsActive()) {
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            intakeTriggers.check();

            g2.readButtons();

            hardware.telemetry.addData("Pose", intake.extension.getPosition());
            hardware.telemetry.update();
        }
    }
}
