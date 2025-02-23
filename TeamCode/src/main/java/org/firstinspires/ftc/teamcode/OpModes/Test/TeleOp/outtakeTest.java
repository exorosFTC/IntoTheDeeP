package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;

@TeleOp(group = "test")
public class outtakeTest extends LinearOpMode {
    Outtake outtake;
    TriggerManager outtakeTriggers;
    Hardware hardware;
    GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(this);
        hardware = Hardware.getInstance(this);
        g2 = new GamepadEx(gamepad2);

        outtakeTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> {
                            outtake.openClaw(false);

                            try { Thread.sleep(300); } catch (InterruptedException e) {}

                            outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SAMPLES);
                            outtake.extendPosition(Enums.OuttakeEnums.LiftAction.HIGH_BASKET);})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> {
                            outtake.openClaw(false);

                            try { Thread.sleep(300); } catch (InterruptedException e) {}

                            outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SPECIMENS);
                            outtake.extendPosition(Enums.OuttakeEnums.LiftAction.HIGH_RUNG);})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.A),
                        () -> {
                            outtake.openClaw(true);

                            try { Thread.sleep(300); } catch (InterruptedException e) {}

                            outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_TRANSFER);
                            outtake.extendPosition(Enums.OuttakeEnums.LiftAction.TRANSFER);
                        })

                .addAction(() -> {
                    outtake.extension.extend(g2.getLeftY() * 0.03);
                    outtake.extension.update();
                });

        waitForStart();

        while (opModeIsActive()) {
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            outtakeTriggers.check();

            g2.readButtons();
            outtake.extension.update();

            hardware.telemetry.addData("Pose", outtake.extension.getPosition());
            hardware.telemetry.update();
        }
    }
}
