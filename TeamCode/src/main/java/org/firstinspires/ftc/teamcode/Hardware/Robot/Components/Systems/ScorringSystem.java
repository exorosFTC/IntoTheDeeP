package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

public class ScorringSystem implements Enums, Enums.IntakeEnums, Enums.OuttakeEnums {
    private static Hardware hardware;

    public Intake intake;
    public Outtake outtake;

    public ScorringSystem(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

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

            outtake.setArmAction(ArmAction.PRE_SCORE);
            intake.setAction(IntakeAction.TRANSFER);
            intake.extendPosition(IntakePosition.ZERO);

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
