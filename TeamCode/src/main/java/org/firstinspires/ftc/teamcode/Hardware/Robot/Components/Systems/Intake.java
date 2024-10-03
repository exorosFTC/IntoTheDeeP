package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeDistance;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeDropdown;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionTouch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeLeftServo;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeRightServo;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeStopper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class Intake implements Enums {
    private static Hardware hardware;

    private static LinearOpMode opMode;

    private final ColorEx red = new ColorEx(255, 0, 0);
    private final ColorEx blue = new ColorEx(0, 0, 255);

    private IntakeAction lastAction = IntakeAction.INIT;

    private final double inIntakeTreshold = 0;
    private final double overBarThreshold = 0;
    private final double encoderMaxTicks = 0;

    private boolean isIntakeAccessible = true;
    private boolean isInPit = false;

    private double extensionPower = 0;

    private static final double
            dropdownInit = 0,
            dropdownTransfer = 0,
            dropdownIntake = 0;

    private static final double
            intakeStopPower = 0.5,
            intakeSpitPower = 0,
            intakeStartPower = 1;

    private static final double
            stopperBlocked = 0,
            stopperThrough = 0;



    public Intake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap);

        this.opMode = opMode;
        hardware.CRservos.get(IntakeRightServo).setDirection(DcMotorSimple.Direction.REVERSE);

        setAction(IntakeAction.INIT);
    }



    public void setAction(IntakeAction action) {
        switch (action){
            case INIT: {
                if (hardware.servos.get(IntakeStopper).getPosition() != stopperBlocked)
                    hardware.servos.get(IntakeStopper).setPosition(stopperBlocked);

                if (hardware.servos.get(IntakeDropdown).getPosition() != dropdownInit)
                    hardware.servos.get(IntakeDropdown).setPosition(dropdownInit);

                hardware.CRservos.get(IntakeLeftServo).setPower(intakeStopPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeStopPower);
            } break;

            case SPIT: {
                if (hardware.servos.get(IntakeStopper).getPosition() != stopperBlocked)
                    hardware.servos.get(IntakeStopper).setPosition(stopperBlocked);

                if (hardware.servos.get(IntakeDropdown).getPosition() != dropdownIntake) {
                    hardware.servos.get(IntakeDropdown).setPosition(dropdownIntake);
                    try { wait(600); }
                    catch (InterruptedException e) {}
                }

                hardware.CRservos.get(IntakeLeftServo).setPower(intakeSpitPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeSpitPower);
            } break;

            case STOP: {
                hardware.CRservos.get(IntakeLeftServo).setPower(intakeStopPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeStopPower);

                try { wait(200); }
                catch (InterruptedException e) {}

                if (hardware.servos.get(IntakeStopper).getPosition() != stopperBlocked)
                    hardware.servos.get(IntakeStopper).setPosition(stopperBlocked);

                if (hardware.servos.get(IntakeDropdown).getPosition() != dropdownIntake)
                    hardware.servos.get(IntakeDropdown).setPosition(dropdownIntake);

            } break;

            case COLLECT: {
                if (hardware.servos.get(IntakeStopper).getPosition() != stopperBlocked)
                    hardware.servos.get(IntakeStopper).setPosition(stopperBlocked);

                if (hardware.servos.get(IntakeDropdown).getPosition() != dropdownIntake) {
                    hardware.servos.get(IntakeDropdown).setPosition(dropdownIntake);
                    try { wait(600); }
                    catch (InterruptedException e) {}
                }

                hardware.CRservos.get(IntakeLeftServo).setPower(intakeStartPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeStartPower);
            } break;

            case TRANSFER: {
                if (hardware.servos.get(IntakeStopper).getPosition() != stopperBlocked)
                    hardware.servos.get(IntakeStopper).setPosition(stopperBlocked);

                if (hardware.servos.get(IntakeDropdown).getPosition() != dropdownTransfer) {
                    hardware.servos.get(IntakeDropdown).setPosition(dropdownTransfer);
                    try { wait(600); }
                    catch (InterruptedException e) {}
                }

                hardware.servos.get(IntakeStopper).setPosition(stopperThrough);
                try { wait(600); }
                catch (InterruptedException e) {}

                hardware.CRservos.get(IntakeLeftServo).setPower(intakeStartPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeStartPower);

                try { wait(1500); }
                catch (InterruptedException e) {}

                hardware.CRservos.get(IntakeLeftServo).setPower(intakeStopPower);
                hardware.CRservos.get(IntakeRightServo).setPower(intakeStopPower);

            } break;
        }

        lastAction = action;
    }

    public void setExtensionPower(double power) { extensionPower = power; }

    private void update() {
        if (isIntakeAccessible) {
            checkExtensionLimits();
            checkIntakeColor();
            checkAutoDropdown();
        }

    }

    private void checkIntakeColor() {
        if (hardware.color.get(IntakeColor).getDistance(DistanceUnit.CM) <= inIntakeTreshold
                                            &&
                            lastAction == IntakeAction.COLLECT)
        {
            if ((SystemConstants.autoOnBlue && checkColor() == Color.BLUE)
                                        ||
                    (!SystemConstants.autoOnBlue && checkColor() == Color.RED)
                                        ||
                    checkColor() == Color.NONE) // this is yellow, I hope
            {
                setExtensionPower(-1);
                while (opMode.opModeIsActive() && !hardware.touch.get(IntakeExtensionTouch).isPressed())
                    checkExtensionLimits();

                setAction(IntakeAction.TRANSFER);
            } else setAction(IntakeAction.SPIT);
        }
    }

    private void checkExtensionLimits() {
        if (hardware.motors.get(IntakeMotor).getCurrentPosition() > encoderMaxTicks
                                            ||
                    hardware.touch.get(IntakeExtensionTouch).isPressed())
            extensionPower = 0;
        else hardware.motors.get(IntakeMotor).setPower(extensionPower);
    }

    private void checkAutoDropdown() {
        if (hardware.distance.get(IntakeDistance).getDistance(DistanceUnit.CM) <= overBarThreshold)
            isInPit = !isInPit;

            if (isInPit && SystemConstants.autoDropdown)
                setAction(IntakeAction.COLLECT);
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
        return Color.NONE;
    }


}
