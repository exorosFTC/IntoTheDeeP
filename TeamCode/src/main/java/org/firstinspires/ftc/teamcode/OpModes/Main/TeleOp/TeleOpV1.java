package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@TeleOp(name = "🍓", group = "main")
public class TeleOpV1 extends ExoMode {
    private Machine robot;

    private Thread drivetrainThread;

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.Telemetry.DASHBOARD)
                        .add(Enums.OpMode.TELE_OP)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        InitializeThreads();
    }

    @Override
    protected void WhenStarted() {
        drivetrainThread.start();

        robot.system.reset();
        robot.initComplete();
    }

    @Override
    protected void InitializeThreads() {
        drivetrainThread = new Thread(() -> {
            while (opModeIsActive()) { robot.updateDrive(); }});
    }

    @Override
    protected void Loop() {
        // outtake access
        if (robot.g2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.SCORE_HIGH_BASKET);
            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.SCORE_LOW_BASKET);

            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.SCORE_HIGH_RUNG);
            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.SCORE_LOW_RUNG);

            if (robot.g2.wasJustPressed(GamepadKeys.Button.X))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.INIT);
            if (robot.g2.wasJustPressed(GamepadKeys.Button.A))
                robot.system.outtake.setAction(Enums.Outtake.OuttakeAction.COLLECT);
            if (robot.g2.wasJustPressed(GamepadKeys.Button.B))
                robot.system.outtake.toggleClaw();

            robot.system.outtake.extend(-robot.g2.getLeftY());
        }

        // intake access
        else if (robot.g2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

            if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                robot.system.intake.setAction(Enums.IntakeAction.INIT);
            else if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                robot.system.intake.setAction(Enums.IntakeAction.COLLECT);

            if (robot.g2.wasJustPressed(GamepadKeys.Button.A))
                robot.system.intake.setAction(Enums.IntakeAction.SPIT);

            robot.system.intake.extend(-robot.g2.getLeftY());
        }

        robot.updateSystem();
        robot.updateTelemetry();
        robot.bulk.clearCache(Enums.Hubs.ALL);
    }

}
