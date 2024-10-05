package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.inTransferThreshold;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionTouch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.TransferDistance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

public class ScorringSystem {
    private static Hardware hardware;

    public Intake intake;
    public Outtake outtake;

    public ScorringSystem(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap);

        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

    }


    public void update() {
        intake.update();

        checkAutoTransfer();
        checkIntakeExtension();
    }

    public void reset() {
        intake.setAction(Enums.IntakeAction.INIT);
        outtake.setAction(Enums.Outtake.OuttakeAction.INIT);

        intake.extension.reset();
        outtake.extension.reset();
    }



    private void checkAutoTransfer() {
        if (hardware.distance.get(TransferDistance).getDistance(DistanceUnit.CM) > inTransferThreshold
                                                &&
                intake.getAction() == Enums.IntakeAction.TRANSFER
        ) {
            outtake.setAction(Enums.Outtake.OuttakeAction.TRANSFER);

            try { wait(400); } catch (InterruptedException e) {}

            outtake.openClaw(false);

            try { wait(200); } catch (InterruptedException e) {}

            outtake.setAction(Enums.Outtake.OuttakeAction.INIT);

            try { wait(300); } catch (InterruptedException e) {}

            intake.setAction(Enums.IntakeAction.INIT);

            try { wait(300); } catch (InterruptedException e) {}

            intake.setAction(Enums.IntakeAction.DISABLE);

        }
    }

    private void checkIntakeExtension() {
        if (!hardware.touch.get(IntakeExtensionTouch).isPressed()
                                    &&
            outtake.getAction() == Enums.Outtake.OuttakeAction.INIT)
            outtake.setAction(Enums.Outtake.OuttakeAction.DISABLE);

    }


}
