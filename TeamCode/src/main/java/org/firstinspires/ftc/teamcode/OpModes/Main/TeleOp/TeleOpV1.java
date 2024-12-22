package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.distance_sensorToClaw;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@Config
@TeleOp(name = "🍓", group = "main")
public class TeleOpV1 extends ExoMode implements Enums.IntakeEnums, Enums.OuttakeEnums {
    private Machine robot;

    public static int toSensor = distance_sensorToClaw;
    private Enums.Access access = Enums.Access.INTAKE;

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.OpMode.TELE_OP)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false)
                        .setUsingAcceleration(true)
                        .setUsingExponentialInput(true))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        InitializeThreads();
        robot.initComplete();
    }

    @Override
    protected void WhenStarted() {
        robot.drivetrainThread.start();

        robot.system.outtake.setAction(OuttakeAction.PRE_TRANSFER);
        robot.system.intake.setAction(IntakeAction.PRE_COLLECT);
    }

    @Override
    protected void InitializeThreads() {

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

                    if (robot.system.intake.getAction() != IntakeAction.PRE_COLLECT && robot.system.intake.hasGameElement()) {
                        robot.system.transfer();

                        access = Enums.Access.OUTTAKE;
                        robot.setAccess(Enums.Access.OUTTAKE);
                    }
                }

                /** rotating the intake with triggers*/
                robot.system.intake.rotate(
                        robot.g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                        robot.g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        0.052);

                /** extend the intake*/
                double input = exp(robot.g2.getLeftY());
                robot.system.intake.extend((Math.abs(input) <= 0.5) ? input * 0.3 : input);
            } break;

            case OUTTAKE: {
                /** DPAD_UP extends the lift into the SAMPLE SCORING position*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    SystemConstants.outtakeScore = ArmAction.SCORE_SAMPLES;
                    robot.system.outtake.setAction(OuttakeAction.PRE_SCORE);
                }

                /** DPAD_LEFT extends the lift into the SPECIMEN SCORING position*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    SystemConstants.outtakeScore = ArmAction.SCORE_SPECIMENS;
                    robot.system.outtake.setAction(OuttakeAction.PRE_SCORE);
                }


                /** A to get back into TRANSFER*/
                if (robot.g2.wasJustPressed(GamepadKeys.Button.A)) {
                    robot.system.outtake.setAction(OuttakeAction.PRE_TRANSFER);

                    access = Enums.Access.INTAKE;
                    robot.setAccess(Enums.Access.INTAKE);
                }

                /** extend the outtake*/
                robot.system.outtake.extend(exp(robot.g2.getLeftY() * 0.25));
            } break;
        }


        if (robot.g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            robot.system.hangG2();


        robot.addTelemetry("OuttakeAction: ", robot.system.outtake.getAction());
        robot.addTelemetry("IntakeAction: ", robot.system.intake.getAction());

        robot.addTelemetry("HasGameElement: ", robot.system.intake.hasGameElement());
        robot.addTelemetry("Access: ", access);

        robot.hardware.bulk.clearCache(Enums.Hubs.ALL);

        robot.debuggingTelemetry();
        robot.updateSystem();
        robot.updateTelemetry();

    }


    private double exp(double x) {
        return x * x * x;
    }

}