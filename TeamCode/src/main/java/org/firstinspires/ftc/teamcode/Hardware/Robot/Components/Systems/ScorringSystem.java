package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

public class ScorringSystem implements Enums, Enums.IntakeEnums, Enums.OuttakeEnums {
    private static Hardware hardware;
    private LinearOpMode opMode;

    public Intake intake;
    public Outtake outtake;

    public ScorringSystem(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

    }

    // our transfer is better .
    public void transfer() {
            outtake.setAction(OuttakeAction.TRANSFER);

            try { Thread.sleep(150); } catch (InterruptedException e) {}

            intake.openClaw(true);
            intake.extendPosition(IntakePosition.IN_TRANSFER);

            try { Thread.sleep(100); } catch (InterruptedException e) {}

            intake.setAction(IntakeAction.PRE_COLLECT);

            try { Thread.sleep(150); } catch (InterruptedException e) {}

            outtake.inverseKinematics(0);
            outtake.setArmAction(ArmAction.PRE_SCORE);
            hardware.servos.get(OuttakeWrist).setPosition(0.2);

            intake.setAction(IntakeAction.TRANSFER);
            intake.extendPosition(IntakePosition.ZERO);
    }

    public void hang() {
        intake.setAction(IntakeAction.TRANSFER);
        intake.extension.disable();
        intake.setAction(IntakeAction.DISABLE);

        hardware.motors.get(LeftFront).setMotorDisable();
        hardware.motors.get(LeftBack).setMotorDisable();
        hardware.motors.get(RightFront).setMotorDisable();
        hardware.motors.get(RightBack).setMotorDisable();

        outtake.setArmAction(ArmAction.PRE_TRANSFER);

        try { Thread.sleep(150); } catch (InterruptedException e) {}

        outtake.setArmAction(ArmAction.DISABLE);
        outtake.extension.setPosition(LiftAction.HANG.name());


    }

    public void update() {
        if (intake.hasJustChangedTo(IntakeAction.PRE_COLLECT))
            outtake.setAction(Outtake.OuttakeAction.PRE_TRANSFER);
        outtake.update();
    }

    public void reset() {
        intake.setAction(IntakeAction.TRANSFER);
        outtake.setAction(Outtake.OuttakeAction.INIT);

        intake.extension.autoReset();
        outtake.extension.autoReset();
    }


}
