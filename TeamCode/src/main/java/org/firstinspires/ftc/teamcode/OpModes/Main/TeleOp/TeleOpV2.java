package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeTurret;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "✨ 🅻🍓🆅🅴 ✨", group = "main")
public class TeleOpV2 extends ExoMode implements Enums.IntakeEnums, Enums.OuttakeEnums {
    private Machine robot;


    private TriggerManager intakeTriggers, outtakeTriggers;
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





        intakeTriggers = new TriggerManager()
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                            () -> {
                    if (robot.system.intake.getAction() != IntakeAction.COLLECT)
                        robot.system.intake.setAction(IntakeAction.COLLECT);
                    else robot.system.intake.setAction(IntakeAction.MOTOR_COLLECT);
                })
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                            () -> robot.system.liftIntake())

                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.B),
                            () -> robot.system.intake.setAction(IntakeAction.MOTOR_STOP))

                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.A),
                            () -> {
                    if (robot.system.intake.getAction() == IntakeAction.COLLECT)
                            robot.system.lowerIntakeAndSpit();
                    else robot.system.collectSpecimensIntakeAccess();
                })


                .addAction(() -> robot.system.intake.rotate(
                        (       robot.g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                                                         -
                                robot.g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)),
                                0.015))
                .addAction(() -> robot.system.intake.extend(robot.g2.getLeftY() * 0.9));



        outtakeTriggers = new TriggerManager()
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                            () -> robot.system.samplesHigh())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                            () -> {
                                if (robot.system.outtake.getAction() == OuttakeAction.COLLECT_SPECIMENS)
                                    robot.system.specimens();
                                else robot.system.throwSampleFromBack();
                })
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.A),
                            () -> robot.system.outtakeToTransfer())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.Y),
                            () -> robot.system.whenTransferFails())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.B),
                            () -> robot.system.score())

                .addAction(() -> {
                    robot.system.outtake.extension.extend(robot.g2.getLeftY() * 0.03);
                });



        robot.initComplete();

        robot.system.intake.manual = true;
        robot.system.outtake.manual = false;
        SystemConstants.updateOuttake = false;
    }

    @Override
    protected void WhenStarted() {
        robot.drivetrainThread.start();

        robot.system.outtake.setAction(OuttakeAction.PRE_TRANSFER);
        robot.system.intake.setAction(IntakeAction.TRANSFER);

        //comment these in real matches pls
        /*robot.system.outtake.extension.resetEncoders();

        robot.system.intake.extension.resetEncoders();
        robot.system.intake.extension.runToPosition();*/
    }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {
        if (robot.system.intake.manual)
            intakeTriggers.check();
        else if (robot.system.outtake.manual)
            outtakeTriggers.check();



        robot.hardware.telemetry.addData("Outtake AMPS", robot.hardware.motors.get(LeftOuttakeMotor).getCurrent(CurrentUnit.AMPS));
        robot.hardware.telemetry.addData("Intake AMPS", robot.hardware.motors.get(IntakeExtensionMotor).getCurrent(CurrentUnit.AMPS));
        robot.hardware.telemetry.addData("update out: ", SystemConstants.updateOuttake);

        robot.hardware.telemetry.addData("Intake pose:", robot.system.intake.extension.getPosition());
        robot.hardware.telemetry.addData("Outtake pose:", robot.system.outtake.extension.getPosition());

        robot.hardware.telemetry.addData("Intake state: ", robot.system.intake.getAction());
        robot.hardware.telemetry.addData("Outtake state: ", robot.system.outtake.getAction());





        //update stuff
        robot.g2.readButtons();
        robot.updateTelemetry();
        robot.system.uptateAutoTransfer();
        robot.system.outtake.extension.update();
        robot.hardware.bulk.clearCache(Enums.Hubs.ALL);
    }
}