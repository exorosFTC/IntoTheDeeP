package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastTurn;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowTurn;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.blue;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.inIntakeThreshold;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.red;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.rotationMax;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.rotationMin;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.v4bSafeDropdown;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.yellow;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.driveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionTouch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeRotation;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeV4B;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.OneMotorLift;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

import java.util.concurrent.TimeUnit;

public class Intake implements Enums, Enums.IntakeEnums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static Intake.IntakeAction previousAction = IntakeAction.TRANSFER, currentAction;
    public final OneMotorLift extension;

    private boolean isIntakeAccessible = true;
    private boolean currentTouch = false, lastTouch = false;

    private static final double
            clawTransferOpen = 0.35,
            clawCollectOpen = 0.32,
            clawClosed = 0.79;

    private static final double
            v4bTransfer = 0.19,
            v4bPreCollect = 0.67,
            v4bCollect = 0.825;

    private static final double
            rotationTransfer = 0.1;



    public Intake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

        hardware.servos.get(IntakeRotation).setDirection(Servo.Direction.REVERSE);

        this.opMode = opMode;
        extension = new OneMotorLift(opMode, IntakeMotor);

        extension.addLimit(extendoMAX)
                .addCurrentAlert(4)
                .reverse();

        for (Intake.IntakePosition action : Intake.IntakePosition.values()) {
            extension.add(action.name(), action.ticks);
        }

    }



    public void setAction(IntakeAction action) {
        switch (action){
            case DISABLE: {
                hardware.servos.get(IntakeClaw).getController().close();
                hardware.servos.get(IntakeRotation).getController().close();
            } break;

            case TRANSFER: {
                hardware.servos.get(IntakeRotation).setPosition(rotationTransfer);
                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(IntakeV4B).setPosition(v4bTransfer);

            } break;

            case PRE_COLLECT: {
                    hardware.servos.get(IntakeV4B).setPosition(v4bPreCollect);

                    if (!hasGameElement())
                        hardware.servos.get(IntakeClaw).setPosition(clawCollectOpen);
            } break;

            case COLLECT: {
                if (extension.getPosition() < v4bSafeDropdown || previousAction == IntakeAction.COLLECT)
                    break;

                if (hasGameElement()) {
                    setAction(IntakeAction.TRANSFER);
                    hardware.servos.get(IntakeClaw).setPosition(clawCollectOpen);
                    break;
                }

                hardware.servos.get(IntakeV4B).setPosition(v4bCollect);
                try { Thread.sleep(150); } catch (InterruptedException e) {}

                hardware.servos.get(IntakeClaw).setPosition(clawClosed);
                try { Thread.sleep(250); } catch (InterruptedException e) {}

                if (checkIntakeCatch()) {
                    hardware.servos.get(IntakeRotation).setPosition(rotationTransfer);
                    hardware.servos.get(IntakeV4B).setPosition(v4bTransfer);

                    extendUntilTouch();
                } else {
                    hardware.servos.get(IntakeClaw).setPosition(clawCollectOpen);
                    hardware.servos.get(IntakeV4B).setPosition(v4bPreCollect);
                }
            } break;
        }

        previousAction = currentAction;
        currentAction = action;
    }




    public void extend(double power, boolean noLimit) {
        extension.extend(power, noLimit);
    }

    public void extend(double power) {
        lastTouch = currentTouch;
        currentTouch = hardware.touch.get(IntakeExtensionTouch).isPressed();
        if (!(currentTouch && power < 0))
            extension.extend(power);
        else if (currentTouch && !lastTouch) {
            extension.resetEncoders();
        }

        if (extension.getPosition() > v4bSafeDropdown && previousAction != IntakeAction.PRE_COLLECT) {
            driveSensitivity = slowTurn;
            setAction(IntakeAction.PRE_COLLECT);
        }
        else if (extension.getPosition() < v4bSafeDropdown && previousAction != IntakeAction.TRANSFER) {
            driveSensitivity = fastTurn;
            setAction(IntakeAction.TRANSFER);
        }
    }

    public void extendPosition(IntakePosition position) { extension.setPosition(position.name()); }

    public void extendUntilTouch() {
        extendUntilTouch(1500);
    }

    public void extendUntilTouch(int ms) {
        ElapsedTime failSwitch = new ElapsedTime();
        int time = (int) (ms * extension.getPosition() / extendoMAX);

        while (!hardware.touch.get(IntakeExtensionTouch).isPressed() && opMode.opModeIsActive() && failSwitch.time(TimeUnit.MILLISECONDS) < time) {
            extension.extend(-1);
            hardware.bulk.clearCache(Hubs.ALL);
        }

        opMode.gamepad2.rumble(1, 1, 700);
        extension.resetEncoders();
    }




    public void rotate(double left, double right, double sensitivity) {
        rotate(left, right, sensitivity, true);
    }

    public void rotate(double left, double right, double sensitivity, boolean exponential) {
        if (currentAction != IntakeAction.TRANSFER) {

            double diff = (exponential) ? (exponential(right) - exponential(left)) : (right - left);
            double position = hardware.servos.get(IntakeRotation).getPosition()
                    +
                    diff * sensitivity;

            double clipped_position = Math.max(rotationMin, Math.min(rotationMax, position));
            hardware.servos.get(IntakeRotation).setPosition(clipped_position);
        }
    }

    public void openClawTransfer(boolean open) {
        hardware.servos.get(IntakeClaw).setPosition((open) ? clawTransferOpen : clawClosed);
    }

    public void toggleClaw() {
        if (hardware.servos.get(IntakeClaw).getPosition() == clawTransferOpen)
            openClawTransfer(false);
        else openClawTransfer(true);
    }



    private double exponential(double x) { return x * x * x; }



    private boolean checkIntakeCatch() {
        if (hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM) > inIntakeThreshold)
            return false;

        ColorEx input = new ColorEx(
                hardware.color.get(IntakeColor).red(),
                hardware.color.get(IntakeColor).green(),
                hardware.color.get(IntakeColor).blue());


        if ((SystemConstants.autoOnBlue && checkColor(input) != Color.RED)
                                        ||
                (!SystemConstants.autoOnBlue && checkColor(input) != Color.BLUE))
            return true;

        return false;
    }

    private Enums.Color checkColor(ColorEx color) {
        if (color.sameAs(blue))
            return Color.BLUE;
        if (color.sameAs(red))
            return Color.RED;
        if (color.sameAs(yellow))
            return Color.YELLOW;
        return Color.NONE;
    }



    public IntakeAction getAction() { return currentAction; }

    public  IntakeAction getPreviousAction() { return previousAction; }

    public double getCurrent() { return  hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS); }

    public boolean hasJustChangedTo(IntakeAction action) { return action == currentAction && action != previousAction; }

    public boolean hasGameElement() { return hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM) <= inIntakeThreshold; }

    public boolean isOverCurrent() { return hardware.motors.get(IntakeMotor).isOverCurrent(); }
}
