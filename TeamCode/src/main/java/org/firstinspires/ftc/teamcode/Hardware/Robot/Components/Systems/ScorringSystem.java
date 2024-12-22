package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeV4B;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

import java.util.concurrent.TimeUnit;

public class ScorringSystem implements Enums, Enums.IntakeEnums, Enums.OuttakeEnums {
    private static Hardware hardware;
    private LinearOpMode opMode;
    private ElapsedTime timer;

    public Intake intake;
    public Outtake outtake;

    public ScorringSystem(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();
        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

    }

    // our transfer is better .
    public void transfer() {
            outtake.setAction(OuttakeAction.TRANSFER);

            //try { Thread.sleep(100); } catch (InterruptedException e) {}

            intake.openClawTransfer(true);
            //intake.extendPosition(IntakePosition.IN_TRANSFER);
            timer.reset();
            while (opMode.opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) <= 80) { opMode.idle(); }
            //try { Thread.sleep(80); } catch (InterruptedException e) {}

            outtake.inverseKinematics(0);
            intake.setAction(IntakeAction.PRE_COLLECT);

            timer.reset();
            while (opMode.opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) <= 100) { opMode.idle(); }

            //try { Thread.sleep(100); } catch (InterruptedException e) {}

            outtake.setArmAction(ArmAction.PRE_SCORE);
            hardware.servos.get(OuttakeWrist).setPosition(0.2);
            intake.setAction(IntakeAction.TRANSFER);


            //intake.setAction(IntakeAction.DISABLE);
            //intake.extendUntilTouch(150);
    }

    public void hangG1() {
        intake.setAction(IntakeAction.DISABLE);
        hardware.servos.get(IntakeV4B).setPosition(0.19);

        hardware.motors.get(LeftFront).setMotorDisable();
        hardware.motors.get(LeftBack).setMotorDisable();
        hardware.motors.get(RightFront).setMotorDisable();
        hardware.motors.get(RightBack).setMotorDisable();

        outtake.setArmAction(ArmAction.PRE_TRANSFER);

        try { Thread.sleep(150); } catch (InterruptedException e) {}

        outtake.setArmAction(ArmAction.DISABLE);
        outtake.extension.setPosition(LiftAction.HANG.name());


    }

    public void hangG2() {
        intake.extendUntilTouch();
        intake.setAction(IntakeAction.TRANSFER);

        outtake.extension.setPosition(outtakeMAX);
        outtake.setArmAction(ArmAction.PRE_TRANSFER);
    }


    public void update() {
        if (intake.hasJustChangedTo(IntakeAction.PRE_COLLECT))
            outtake.setAction(Outtake.OuttakeAction.PRE_TRANSFER);
        outtake.extension.update();
    }


}
