package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@TeleOp(name = "🥰🥰🥰🥰🥰", group = "main")
public class TeleOpSolo extends ExoMode {
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
                .addGamepads(Enums.Gamepads.G2)
                .construct(this);





        intakeTriggers = new TriggerManager()
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> {
                            if (robot.system.intake.getAction() != Enums.IntakeEnums.IntakeAction.COLLECT)
                                robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);
                            else robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_COLLECT);
                        })
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> robot.system.liftIntake())

                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.B),
                        () -> robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_STOP))

                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.A),
                        () -> {
                            if (robot.system.intake.getAction() == Enums.IntakeEnums.IntakeAction.COLLECT)
                                robot.system.lowerIntakeAndSpit();
                            else robot.system.collectSpecimensIntakeAccess();
                        })


                .addTrigger(() -> robot.g2.gamepad.touchpad_finger_1,
                        () -> {
                            double raw = (gamepad2.touchpad_finger_1_x + 1) / 2;
                            int position = (int) (raw * SystemConstants.extendoMAX);

                            robot.system.intake.extension.setPosition(position);
                        })

                


                .addAction(() -> robot.system.intake.rotate(
                        (       robot.g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                                -
                                robot.g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)),
                        0.015));

        outtakeTriggers = new TriggerManager()
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> robot.system.samplesHigh())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> {
                            if (robot.system.outtake.getAction() == Enums.OuttakeEnums.OuttakeAction.COLLECT_SPECIMENS)
                                robot.system.specimens();
                            else robot.system.throwSampleFromBack();
                        })
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.A),
                        () -> robot.system.outtakeToTransfer())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.Y),
                        () -> robot.system.whenTransferFails())
                .addTrigger(() -> robot.g2.wasJustPressed(GamepadKeys.Button.B),
                        () -> robot.system.score());



        robot.initComplete();


        robot.system.intake.manual = true;
        robot.system.outtake.manual = false;

        SystemConstants.updateOuttake = false;
        extendoMAX = 670;

        robot.system.outtake.extension.resetEncoders();
        robot.system.intake.extension.resetEncoders();
        robot.system.intake.extension.runToPosition();
    }

    @Override
    protected void WhenStarted() {
        robot.drivetrainThread.start();

        robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_TRANSFER);
        robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);

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
        robot.hardware.telemetry.addData("Collection Motor AMPS", robot.hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS));
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
