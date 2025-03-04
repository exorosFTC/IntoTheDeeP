package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.waitReachedOuttake;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.AnyMotorLift;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class Outtake implements Enums.OuttakeEnums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static OuttakeAction previousAction, currentAction;
    public final AnyMotorLift extension;
    private final ElapsedTime timer;


    public static final double
            armTransfer = 0.033,
            armPreTransfer = 0.01,
            armScoreSpecimens = 0.36,
            armScoreSamples = 0.63,
            armCollectSpecimens = 1;

    public static final double
            wristTransfer = 0,
            wristScoreSpecimens = 0.17,
            wristScoreSamples = 0.17,
            wristCollectSpecimens = 0.3;

    public static final double
            clawOpen = 0.15,
            clawClosed = 0.45;



    private final double p = 0.01, d = 0, f = 0, alpha = 0.3;
    public boolean manual = false;


    public Outtake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        hardware.servos.get(OuttakeLeftPivot).setDirection(Servo.Direction.REVERSE);

        timer = new ElapsedTime();

        this.opMode = opMode;
        this.extension = new AnyMotorLift(opMode,
                Arrays.asList(RightOuttakeMotor, LeftOuttakeMotor),
                LeftOuttakeMotor);

        for (LiftAction action : LiftAction.values()) {
            extension.add(action.name(), action.ticks);
        }

        extension.addLimit(outtakeMAX)
                .addCurrentAlert(6.5)
                .setControllerPID(p, 0, d, f)
                .setAlpha(alpha)
                .reverse(RightOuttakeMotor);
    }



    public void moveArm(double position) {
        hardware.servos.get(OuttakeLeftPivot).setPosition(position);
        hardware.servos.get(OuttakeRightPivot).setPosition(position);
    }

    public void openClaw(boolean open) {
        hardware.servos.get(OuttakeClaw).setPosition((open) ? clawOpen : clawClosed);
    }

    public void setAction(OuttakeAction action) {
        switch (action) {
            case PRE_TRANSFER: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armPreTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armPreTransfer);
                hardware.servos.get(OuttakeWrist).setPosition(wristTransfer);

            } break;

            case SCORE_SAMPLES: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armScoreSamples);
                hardware.servos.get(OuttakeRightPivot).setPosition(armScoreSamples);
                hardware.servos.get(OuttakeWrist).setPosition(wristScoreSamples);
            } break;

            case SCORE_SPECIMENS: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armScoreSpecimens);
                hardware.servos.get(OuttakeRightPivot).setPosition(armScoreSpecimens);
                hardware.servos.get(OuttakeWrist).setPosition(wristScoreSpecimens);
            } break;

            case COLLECT_SPECIMENS: {
                moveArm(0.6);

                try { Thread.sleep(70); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeWrist).setPosition(wristCollectSpecimens);

                try { Thread.sleep(370); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeLeftPivot).setPosition(armCollectSpecimens);
                hardware.servos.get(OuttakeRightPivot).setPosition(armCollectSpecimens);

                try { Thread.sleep(50); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);
            } break;

        }
        previousAction = currentAction;
        currentAction = action;
    }



    public void extendPosition(Enums.OuttakeEnums.LiftAction position) { extension.setPosition(position.name()); }

    public void extendUntilZero() {
        double failSafeTime = extension.getPosition() * 3;

        extension.setPosition(0);
        timer.reset();

        while (opMode.opModeIsActive()
                &&
                extension.getPosition() > 6
                &&
                !extension.constrained()
                &&
                timer.time(TimeUnit.MILLISECONDS) < failSafeTime)
        {
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            extension.update();

        }
    }

    public void waitReached(Enums.OuttakeEnums.LiftAction position) {
        double failSafeTime = Math.abs(extension.getPosition() - position.ticks) * 3;
        extension.setPosition(position.name());
        timer.reset();

        while (opMode.opModeIsActive()
                &&
                !extension.reached(waitReachedOuttake)
                &&
                !extension.constrained()
                &&
                timer.time(TimeUnit.MILLISECONDS) < failSafeTime)
        {
            hardware.telemetry.addData("outtake position: ", extension.getPosition());
            hardware.telemetry.addData("outtake current: ", getCurrent());

            if (opModeType != Enums.OpMode.AUTONOMUS) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
                hardware.telemetry.update();
                extension.update();
            }
        }
    }



    public OuttakeAction getAction() { return currentAction; }

    public OuttakeAction getPreviousAction() { return previousAction; }

    public boolean getCurrent() { return hardware.motors.get(LeftOuttakeMotor).isOverCurrent(); }



    public boolean hasJustChangedTo(OuttakeAction action) { return action == currentAction && action != previousAction; }

}
