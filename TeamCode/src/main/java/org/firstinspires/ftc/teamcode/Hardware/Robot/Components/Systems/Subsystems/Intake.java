package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.blue;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.inIntakeThreshold;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.isExtensionOut;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.red;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.turretMax;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.turretMin;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.waitReachedIntake;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.yellow;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeExtensionMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeLocker;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeTurret;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeWrist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.OneMotorLift;

import java.util.concurrent.TimeUnit;

public class Intake implements Enums, Enums.IntakeEnums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static Intake.IntakeAction previousAction = IntakeAction.TRANSFER,
                                        currentAction = IntakeAction.TRANSFER;
    public final OneMotorLift extension;
    private final ElapsedTime timer;



    public static final double
            turretTransfer = 0.92,
            turretCollect = 0.37;

    public static final double
            wristTransfer = 1,
            wristUp = 1,
            wristCollect = 0.66;

    public static final double
            lockerOpen = 1,
            lockerClosed = 0.33;



    public boolean isOut = false;
    public boolean manual = true;



    public Intake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();

        extension = new OneMotorLift(opMode, IntakeExtensionMotor);
        extension.addLimit(extendoMAX)
                .addCurrentAlert(8)

                .reverse();

        for (Intake.IntakePosition action : Intake.IntakePosition.values()) {
            extension.add(action.name(), action.ticks);
        }

    }


    // for later use: try { Thread.sleep(); } catch (InterruptedException e) {}
    public void setAction(IntakeAction action) {
        switch (action){
            case TRANSFER: {
                isOut = false;

                hardware.motors.get(IntakeMotor).setPower(0);

                hardware.servos.get(IntakeWrist).setPosition(wristUp);
                try { Thread.sleep(300); } catch (InterruptedException e) {}
                hardware.servos.get(IntakeTurret).setPosition(turretTransfer);

            } break;

            case COLLECT: {
                if (isOut) {
                    hardware.servos.get(IntakeWrist).setPosition(wristCollect);
                    break;
                }
                isOut = true;

                hardware.servos.get(IntakeTurret).setPosition(turretCollect);

                try { Thread.sleep(300); } catch (InterruptedException e) {}

                hardware.servos.get(IntakeWrist).setPosition(wristCollect);
                hardware.servos.get(IntakeLocker).setPosition(lockerOpen);
            } break;

            case BACK_UP: {
                if (!isOut) break;
                hardware.servos.get(IntakeWrist).setPosition(wristUp);
            } break;



            case MOTOR_COLLECT: { hardware.motors.get(IntakeMotor).setPower(1); } break;

            case MOTOR_SPIT: { hardware.motors.get(IntakeMotor).setPower(-0.6); } break;

            case MOTOR_STOP: { hardware.motors.get(IntakeMotor).setMotorDisable(); } break;
        }

        if (action != IntakeAction.MOTOR_COLLECT && action != IntakeAction.MOTOR_SPIT && action != IntakeAction.MOTOR_STOP) {
            previousAction = currentAction;
            currentAction = action;
        }
    }




    public void extend(double power, boolean noLimit) { extension.extend(power, noLimit);}

    public void extend(double power) { extension.extend(power); }

    public void extendPosition(IntakePosition position) { extension.setPosition(position.name()); }

    public void extendUntilZero() {
        // "apoi direct cu visinata pasta tasta.. doamne f***-o si tastatura" - quote of the day
        double failSafeTime = extension.getPosition() * 2.5;

        extension.runWithoutEncoder();
        extension.extend(-1);
        timer.reset();

        while (opMode.opModeIsActive()
                        &&
                extension.getPosition() > 2
                        &&
                timer.time(TimeUnit.MILLISECONDS) < failSafeTime)
        {
            hardware.telemetry.addData("intake position: ", extension.getPosition());
            hardware.telemetry.addData("intake current: ", extension.getCurrent());
            hardware.telemetry.update();

            if (opModeType != Enums.OpMode.AUTONOMUS) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        }

        extension.extend(0);
        extension.setPosition(0);
        extension.runToPosition();
        extension.disable();
    }

    public void waitReached(int position) {
        double failSafeTime = (extension.getPosition() - position) * 2.5;

        extension.setPosition(position);
        timer.reset();

        while (opMode.opModeIsActive()
                &&
                extension.reached(waitReachedIntake)
                &&
                timer.time(TimeUnit.MILLISECONDS) < failSafeTime)
        {
            hardware.telemetry.addData("intake position: ", extension.getPosition());
            hardware.telemetry.addData("intake current: ", extension.getCurrent());
            hardware.telemetry.update();

            if (opModeType != Enums.OpMode.AUTONOMUS) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        }
    }



    public void rotate(double power, double sensitivity) {
        if (!isOut) return;

        double position = hardware.servos.get(IntakeTurret).getPosition()
                                                +
                                        power * sensitivity;

        double wrapped_position = Math.max(turretMin, Math.min(turretMax, position));
        hardware.servos.get(IntakeTurret).setPosition(wrapped_position);

    }


    private double exponential(double x) { return x * x * x; }



    public boolean checkIntakeCatch() {
        double red = hardware.color.get(IntakeColor).red();

        if ((SystemConstants.autoOnBlue && checkColor(red) != Color.RED)
                                        ||
                (!SystemConstants.autoOnBlue && checkColor(red) != Color.BLUE)
                                        ||
                checkColor(red) == Color.YELLOW)
            return true;

        return false;
    }

    // only red thresholding for faster computing
    private Enums.Color checkColor(double r) {
        if (r < 1300)
            return Color.BLUE;
        if (r < 4300 && r > 1500)
            return Color.RED;
        if (r > 6500)
            return Color.YELLOW;
        return Color.NONE;
    }




    public IntakeAction getAction() { return currentAction; }

    public  IntakeAction getPreviousAction() { return previousAction; }

    public double getCurrent() { return  hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS); }





    public boolean hasJustChangedTo(IntakeAction action) { return action == currentAction && action != previousAction; }

    public boolean hasGameElement() { return hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM) <= inIntakeThreshold; }



    public boolean isOverCurrent() { return hardware.motors.get(IntakeMotor).isOverCurrent(); }

    public boolean isOut() { return isOut || extension.getPosition() > isExtensionOut; }

}
