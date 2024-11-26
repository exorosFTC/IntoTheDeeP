package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.blue;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.inIntakeThreshold;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.red;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.rotationMax;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.rotationMin;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.v4bSafeDropdown;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.yellow;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionTouch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeRotation;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeV4B;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.OneMotorLift;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class Intake implements Enums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static IntakeAction lastAction = IntakeAction.TRANSFER;
    public final OneMotorLift extension;

    private boolean isIntakeAccessible = true;

    private static final double
            clawOpen = 0.4,
            clawClosed = 0.78;

    private static final double
            v4bTransfer = 0.8,
            v4bPreCollect = 0.3,
            v4bCollect = 0.1;

    private static final double
            rotationTransfer = 0.08;



    public Intake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap, opMode.telemetry);

        this.opMode = opMode;
        extension = new OneMotorLift(opMode, IntakeMotor);

        extension.addLimit(extendoMAX)
                .addCurrentAlert(1)
                .add("zero", 0)
                .reverse();

    }



    public void setAction(IntakeAction action) {
        switch (action){
            case TRANSFER: {
                if (hardware.servos.get(IntakeRotation).getPosition() != rotationTransfer) {
                    hardware.servos.get(IntakeRotation).setPosition(rotationTransfer);
                    try { Thread.sleep(100); } catch (InterruptedException e) {}
                }

                if (hardware.servos.get(IntakeV4B).getPosition() != v4bTransfer)
                    hardware.servos.get(IntakeV4B).setPosition(v4bTransfer);

            } break;

            case PRE_COLECT: {
                if (hardware.servos.get(IntakeV4B).getPosition() != v4bCollect) {
                    hardware.servos.get(IntakeV4B).setPosition(v4bPreCollect);
                    hardware.servos.get(IntakeClaw).setPosition(clawOpen);
                }
            } break;

            case COLLECT: {
                if (extension.getPosition() < v4bSafeDropdown)
                    break;

                if (hardware.servos.get(IntakeClaw).getPosition() != clawOpen) {
                    hardware.servos.get(IntakeClaw).setPosition(clawOpen);
                    try { Thread.sleep(100); } catch (InterruptedException e) {}
                }

                hardware.servos.get(IntakeV4B).setPosition(v4bCollect);
                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(IntakeClaw).setPosition(clawClosed);
                try { Thread.sleep(100); } catch (InterruptedException e) {}

                if (checkIntakeCatch()) {
                    setAction(IntakeAction.TRANSFER);

                    while (!hardware.touch.get(IntakeExtensionTouch).isPressed() && opMode.opModeIsActive())
                        extension.extend(-1);
                } else setAction(IntakeAction.PRE_COLECT);
            } break;
        }

        lastAction = action;
    }



    public void extend(double power) {
        if (!hardware.touch.get(IntakeExtensionTouch).isPressed())
            extension.extend(power);

        if (extension.getPosition() > v4bCollect)
            setAction(IntakeAction.PRE_COLECT);
        else setAction(IntakeAction.TRANSFER);
    }

    public void rotate(double left, double right, double sensitivity) {
        double position = hardware.servos.get(IntakeRotation).getPosition()
                                            +
                            (left - right) * sensitivity;

        double clipped_position = Math.max(rotationMin, Math.min(rotationMax, position));
        hardware.servos.get(IntakeRotation).setPosition(clipped_position);
    }

    public void openClaw(boolean open) {
        hardware.servos.get(IntakeClaw).setPosition((open) ? clawOpen : clawClosed);
    }

    public void toggleClaw() {
        if (hardware.servos.get(IntakeClaw).getPosition() == clawOpen)
            openClaw(false);
        else openClaw(true);
    }



    private boolean checkIntakeCatch() {
        if (hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM) > inIntakeThreshold)
            return false;

        if ((SystemConstants.autoOnBlue && checkColor() == Color.BLUE)
                                        ||
                (!SystemConstants.autoOnBlue && checkColor() == Color.RED)
                                        ||
                checkColor() == Color.YELLOW)
            return true;

        return false;
    }

    private Enums.Color checkColor() {
        double r,g,b;
        ColorEx color;

        r = hardware.color.get(IntakeColor).red();
        g = hardware.color.get(IntakeColor).green();
        b = hardware.color.get(IntakeColor).blue();

        color = new ColorEx(r, g, b);

        if (color.sameAs(blue))
            return Color.BLUE;
        if (color.sameAs(red))
            return Color.RED;
        if (color.sameAs(yellow))
            return Color.YELLOW;
        return Color.NONE;
    }

    public IntakeAction getAction() { return lastAction; }


}
