package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeExtension;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.TwoMotorLift;

public class Outtake implements Enums.Outtake {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static OuttakeAction lastAction;
    public final TwoMotorLift extension;

    private static final double
            clawOpen = 0,
            clawClosed = 0;

    private static final double
            armCollect = 0,
            armTransfer = 0,
            armScore = 0,
            armHang = 0,
            armInit = 0;

    private static final double
            littleExtensionCollect = 0,
            littleExtensionTransfer = 0,
            littleExtensionScore = 0,
            littleExtensionInit = 0;

    private static final double
            wristCollect = 0,
            wristTransfer = 0,
            wristScore = 0,
            wristHang = 0,
            wristInit = 0;



    public Outtake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap);
        hardware.servos.get(OuttakeLeftPivot).setInverted(true);

        this.opMode = opMode;
        this.extension = new TwoMotorLift(opMode, LeftOuttakeMotor, RightOuttakeMotor);

        for (LiftAction action : LiftAction.values()) {
            extension.add(action.name(), action.ticks);
        }

        extension.addLimit(outtakeMAX)
                .addCurrentAlert(1)
                .reverseRight();
    }



    public void openClaw(boolean open) {
        hardware.servos.get(OuttakeClaw).setPosition((open) ? clawOpen : clawClosed);
    }

    public void toggleClaw() {
        if (hardware.servos.get(OuttakeClaw).getPosition() == clawOpen)
            openClaw(false);
        else openClaw(true);
    }

    private void setArmAction(ArmAction action) {
        switch (action) {
            case INIT: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armInit);
                hardware.servos.get(OuttakeRightPivot).setPosition(armInit);

                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionInit);
                hardware.servos.get(OuttakeWrist).setPosition(wristInit);
                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
            } break;

            case SCORE: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armScore);
                hardware.servos.get(OuttakeRightPivot).setPosition(armScore);

                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionScore);

                try { wait(300); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeWrist).setPosition(wristScore);

                if (hardware.servos.get(OuttakeClaw).getPosition() != clawClosed)
                    hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
            } break;

            case HANG: {
                hardware.servos.get(OuttakeWrist).setPosition(wristHang);
                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionScore); // fully retracted

                try { wait(600); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeLeftPivot).setPosition(armHang);
                hardware.servos.get(OuttakeRightPivot).setPosition(armHang);

                // save current
                hardware.servos.get(OuttakeWrist).disable();
                hardware.servos.get(OuttakeClaw).disable();
                hardware.servos.get(OuttakeExtension).disable();
            } break;

            case COLLECT: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armCollect);
                hardware.servos.get(OuttakeRightPivot).setPosition(armCollect);

                try { wait(400); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeWrist).setPosition(wristCollect);
                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionCollect);

                if (hardware.servos.get(OuttakeClaw).getPosition() != clawOpen)
                    hardware.servos.get(OuttakeClaw).setPosition(clawOpen);

            } break;

            case TRANSFER: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armTransfer);

                hardware.servos.get(OuttakeWrist).setPosition(wristTransfer);

                try { wait(300); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionTransfer);

                if (hardware.servos.get(OuttakeClaw).getPosition() != clawOpen)
                    hardware.servos.get(OuttakeClaw).setPosition(clawOpen);
            } break;

            case DISABLE: {
                hardware.servos.get(OuttakeWrist).setPosition(wristTransfer);
                hardware.servos.get(OuttakeExtension).setPosition(littleExtensionTransfer);

                try { wait(600); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeLeftPivot).setPosition(armTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armTransfer);

                hardware.servos.get(OuttakeWrist).disable();
                hardware.servos.get(OuttakeClaw).disable();
                hardware.servos.get(OuttakeExtension).disable();
            } break;

        }


    }

    private void setLiftAction(LiftAction action) { extension.setPosition(action.name()); }

    public void setAction(OuttakeAction action) {
        switch (action) {
            case DISABLE: {
                setArmAction(ArmAction.DISABLE);
                extension.disable();
            } break;

            case INIT: {
                setArmAction(ArmAction.INIT);

                try { wait(500); } catch (InterruptedException e) {}

                setLiftAction(LiftAction.ZERO);

                try { wait(300); } catch (InterruptedException e) {}

                extension.disable();
            } break;

            case HANG: {
                setLiftAction(LiftAction.FULL);

                try { wait(500); } catch (InterruptedException e) {}

                setArmAction(ArmAction.HANG);
            } break;

            case TRANSFER: {
                if (lastAction == OuttakeAction.COLLECT) {
                    setLiftAction(LiftAction.COLLECT);

                    try { wait(500); } catch (InterruptedException e) {}

                    setArmAction(ArmAction.TRANSFER);
                    setLiftAction(LiftAction.ZERO);

                    try { wait(300); } catch (InterruptedException e) {}

                    extension.disable();

                } else {
                    setArmAction(ArmAction.TRANSFER);

                    try { wait(500); } catch (InterruptedException e) {}

                    setLiftAction(LiftAction.ZERO);

                    try { wait(300); } catch (InterruptedException e) {}

                    extension.disable();
                }
            } break;

            case COLLECT: {
                // wait to not hit anything important (please)
                if (extension.getPosition() <= LiftAction.COLLECT.ticks) {
                    setLiftAction(LiftAction.COLLECT);

                    try { wait(500); } catch (InterruptedException e) {}

                    setArmAction(ArmAction.COLLECT);

                    try { wait(300); } catch (InterruptedException e) {}

                    setLiftAction(LiftAction.ZERO);

                    try { wait(300); } catch (InterruptedException e) {}

                    extension.disable();
                } else {
                    setArmAction(ArmAction.COLLECT);
                    setLiftAction(LiftAction.ZERO);
                    extension.disable();
                }
            } break;


            case SCORE_LOW_RUNG: {
                setArmAction(ArmAction.SCORE);
                setLiftAction(LiftAction.LOW_RUNG);
            } break;

            case SCORE_HIGH_RUNG: {
                setArmAction(ArmAction.SCORE);
                setLiftAction(LiftAction.HIGH_RUNG);
            } break;

            case SCORE_LOW_BASKET: {
                setArmAction(ArmAction.SCORE);
                setLiftAction(LiftAction.LOW_BASKET);
            } break;

            case SCORE_HIGH_BASKET: {
                setArmAction(ArmAction.SCORE);
                setLiftAction(LiftAction.HIGH_BASKET);
            } break;

        }

        lastAction = action;
    }

    public void extend(double power) { extension.extend(power); }

    public OuttakeAction getAction() { return lastAction; }
}
