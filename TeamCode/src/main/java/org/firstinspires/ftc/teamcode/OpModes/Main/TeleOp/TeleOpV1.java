package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@TeleOp(name = "🍓", group = "main")
public class TeleOpV1 extends ExoMode implements Enums.IntakeEnums, Enums.OuttakeEnums {
    private Machine robot;
    private Thread drivetrainThread, outtakeExtensionThread;

    private Enums.Access access = Enums.Access.INTAKE;


    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.Telemetry.REGULAR)
                        .add(Enums.OpMode.TELE_OP)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        InitializeThreads();
        robot.initComplete();
    }

    @Override
    protected void WhenStarted() {
        drivetrainThread.start();
        outtakeExtensionThread.start();

        robot.system.outtake.setAction(OuttakeAction.PRE_TRANSFER);
    }

    @Override
    protected void InitializeThreads() {
        drivetrainThread = new Thread(() -> {
            while (opModeIsActive()) robot.updateDrive();
        });

        outtakeExtensionThread = new Thread(() -> {
            while (opModeIsActive()) {
                robot.system.outtake.update();
                robot.bulk.clearCache(Enums.Hubs.ALL);
            }
        });
    }

    @Override
    protected void Loop() {
        if (robot.g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            access = Enums.Access.INTAKE;
            robot.setAccess(access);
        }

        if (robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            access = Enums.Access.OUTTAKE;
            robot.setAccess(access);
        }


        switch (access) {
            case INTAKE: {
                /** B for COLLECTING from the ground*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.B))
                    robot.system.intake.setAction(IntakeAction.COLLECT);

                /** A for TRANSFERRING the sample/specimen to the outtake*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.A)) {

                    if (robot.system.intake.getAction() != IntakeAction.PRE_COLECT && robot.system.intake.hasGameElement()) {
                        robot.system.transfer();

                        access = Enums.Access.OUTTAKE;
                        robot.setAccess(Enums.Access.OUTTAKE);
                    }
                }

                /** X to TOGGLE the claw*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.X))
                    robot.system.intake.toggleClaw();

                /** rotating the intake with triggers*/
                robot.system.intake.rotate(
                        robot.g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                        robot.g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        0.02);

                /** extend the intake*/
                if (robot.g2.getLeftY() != 0)
                    robot.system.intake.extend(exp(robot.g2.getLeftY()));
            } break;

            case OUTTAKE: {
                /** set RUNG or BASKET scoring*/
                if (robot.g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
                    SystemConstants.outtakeScore = ArmAction.SCORE_SAMPLES;
                else if (robot.g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0)
                    SystemConstants.outtakeScore = ArmAction.SCORE_SPECIMENS;

                /** DPAD_UP extends the lift into SCORING position*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                    robot.system.outtake.setAction(OuttakeAction.PRE_SCORE);
                }

                /** B to actually SCORE*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.B)) {
                    robot.system.outtake.setAction(OuttakeAction.SCORE);
                }

                /** A to get back into TRANSFER*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.A)) {
                    robot.system.outtake.setAction(OuttakeAction.PRE_TRANSFER);

                    access = Enums.Access.INTAKE;
                    robot.setAccess(Enums.Access.INTAKE);
                }

                /** X to TOGGLE the claw*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.X))
                    robot.system.outtake.toggleClaw();

                /** extend the outtake*/
                if (robot.g2.getLeftY() != 0)
                    robot.system.outtake.extend(exp(robot.g2.getLeftY()));
            }

        robot.addTelemetry("OuttakeAction: ", robot.system.outtake.getAction());
        robot.addTelemetry("IntakeAction: ", robot.system.intake.getAction());

        robot.addTelemetry("HasGameElement: ", robot.system.intake.hasGameElement());
        robot.addTelemetry("Access: ", access);

        robot.updateSystem();
        robot.updateTelemetry();
    }

    private double exp(double x) {
        return x * x * x;
    }

}
