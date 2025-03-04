package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.successfulCatch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.updateOuttake;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.waitReachedOuttake;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.FrontUltrasonic;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeLocker;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeTurret;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.UltrasonicSensor;

import java.util.concurrent.TimeUnit;

public class ScorringSystem implements Enums, Enums.IntakeEnums, Enums.OuttakeEnums {
    private static Hardware hardware;
    private LinearOpMode opMode;
    private ElapsedTime timer;

    public Intake intake;
    public Outtake outtake;

    private final UltrasonicSensor front;

    public ScorringSystem(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();
        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

        front = new UltrasonicSensor(opMode, hardware.analog.get(FrontUltrasonic));

    }

    public void lowerIntake() { intake(true, false, false); }
    public void lowerIntakeAndStart() { intake(true, true, false); }
    public void lowerIntakeAndSpit() { intake(true, true, true); }

    public void liftIntake() { intake(false, false, false); }


    private void intake(boolean lower, boolean start, boolean spit) {
        if (!lower) {
            intake.setAction(IntakeAction.BACK_UP);
            intake.setAction(IntakeAction.MOTOR_STOP);

            return;
        }

        intake.setAction(IntakeAction.COLLECT);

        if (start) {
            if (spit) intake.setAction(IntakeAction.MOTOR_SPIT);
            else {
                intake.setAction(IntakeAction.MOTOR_COLLECT);
                hardware.servos.get(IntakeLocker).setPosition(intake.lockerOpen);
            }

        } else intake.setAction(IntakeAction.MOTOR_STOP);
    }




    public void outtakeToTransfer() {
        outtake.openClaw(true);
        outtake.setAction(OuttakeAction.PRE_TRANSFER);
        outtake.moveArm(0.08);


        if (opModeType == OpMode.TELE_OP)
            hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);

        try { Thread.sleep(360); } catch (InterruptedException e) {}

        hardware.servos.get(IntakeTurret).setPosition(0.77);
        outtake.extendUntilZero();

        try { Thread.sleep(100); } catch (InterruptedException e) {}

        outtake.moveArm(0.01);
        try { Thread.sleep(220); } catch (InterruptedException e) {}
        disableOuttake();
    }

    public void collectSpecimensIntakeAccess() {
        hardware.servos.get(IntakeTurret).setPosition(0.72);

        try { Thread.sleep(100); } catch (InterruptedException e) {}

        outtake.openClaw(true);
        outtake.setAction(OuttakeAction.COLLECT_SPECIMENS);

        disableIntake();
        intake.isOut = false;
    }

    public void throwSampleFromBack() {
        updateOuttake = true;
        outtake.extension.setPosition(200);

        timer.reset();
        while (opMode.opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 400 && !outtake.extension.reached(6))
        {
            outtake.extension.update();
            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }

        outtake.extension.disable();
        outtake.setAction(OuttakeAction.COLLECT_SPECIMENS);

        outtakeToTransfer();
        intake.isOut = false;
    }



    // autonomus actions
    public void uptateAutoTransfer() {
        if (hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS) > 8) {
            intake.setAction(IntakeAction.MOTOR_SPIT);
            try { Thread.sleep(200); } catch (InterruptedException e) {}
            intake.setAction(IntakeAction.MOTOR_COLLECT);
        }

        if (!intake.manual || !intake.isOut() || !intake.hasGameElement()) return;

        //close locker and check color
        hardware.servos.get(IntakeLocker).setPosition(intake.lockerClosed);
        intake.setAction(IntakeAction.MOTOR_STOP);

        try { Thread.sleep(70); } catch (InterruptedException e) {}

        if (!intake.checkIntakeCatch()) {
            //spit the wrong color
            hardware.servos.get(IntakeLocker).setPosition(intake.lockerOpen);

            intake.setAction(IntakeAction.MOTOR_SPIT);
            try { Thread.sleep(200); } catch (InterruptedException e) {}
            intake.setAction(IntakeAction.MOTOR_COLLECT);

            return;
        }

        successfulCatch = true;

        if (opModeType == OpMode.TELE_OP)
            transferSequence();
        else transferSequence();
    }

    public void updateAutoTransferAuto() {
        if (!intake.manual || !intake.isOut() || !intake.hasGameElement()) return;

        //close locker and check color
        hardware.servos.get(IntakeLocker).setPosition(intake.lockerClosed);
        intake.setAction(IntakeAction.MOTOR_STOP);

        successfulCatch = true;

        try { Thread.sleep(250); } catch (InterruptedException e) {}

        transferSequence();

    }

    public void waitCatch() {
        if (!intake.manual || !intake.isOut() || !intake.hasGameElement()) return;

        hardware.servos.get(IntakeLocker).setPosition(intake.lockerClosed);
        intake.setAction(IntakeAction.MOTOR_STOP);

        successfulCatch = true;

        try { Thread.sleep(250); } catch (InterruptedException e) {}

        hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);
    }

    public void autoScoreSequence() {
        if (!outtake.manual || outtake.getAction() != OuttakeAction.SCORE_SPECIMENS) return;

        if (front.getFilteredDistance(DistanceUnit.MM) < 20) {
            score();

            // simplifies buttons for the driver in case of multiple specimen cycles one after another.
            outtakeToTransfer();
        }

    }



    // transfer functions
    public void transferSequence() {
        // update access
        intake.manual = false;
        outtake.manual = true;

        // update constants
        SystemConstants.updateOuttake = true;

        // right game element -> starting the transfer sequence
        intake.setAction(IntakeAction.TRANSFER);

        hardware.servos.get(OuttakeWrist).setPosition(outtake.wristTransfer);
        outtake.openClaw(true);


        try { Thread.sleep(350); } catch (InterruptedException e) {}

        outtake.extension.setPosition(40);
        hardware.motors.get(IntakeMotor).setPower(0);
        hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);

        intake.extension.runWithoutEncoder();
        outtake.extension.setPosition(40);
        intake.extension.extend(-1);
        timer.reset();

        while (opMode.opModeIsActive()
                &&
                intake.extension.getPosition() > 10) {
            hardware.bulk.clearCache(Hubs.ALL);
            outtake.extension.update();
        }

        hardware.servos.get(IntakeWrist).setPosition(intake.wristTransfer);
        intake.extension.extend(0);
        intake.extension.runToPosition();
        intake.extension.disable();

        try { Thread.sleep(200); } catch (InterruptedException e) {}

        hardware.servos.get(OuttakeLeftPivot).setPosition(outtake.armTransfer);
        hardware.servos.get(OuttakeRightPivot).setPosition(outtake.armTransfer);

        try { Thread.sleep(100); } catch (InterruptedException e) {}

        outtake.openClaw(false);
        hardware.servos.get(IntakeLocker).setPosition(intake.lockerOpen);

        try { Thread.sleep(170); } catch (InterruptedException e) {}

        disableIntake();
    }

    public void autoTransferSequence() {
        // update access
        intake.manual = false;
        outtake.manual = true;

        // update constants
        SystemConstants.updateOuttake = true;


        hardware.servos.get(OuttakeWrist).setPosition(outtake.wristTransfer);
        outtake.openClaw(true);
        intake.isOut = false;

        hardware.motors.get(IntakeMotor).setPower(0);
        hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);

        outtake.extension.setPosition(40);
        hardware.motors.get(IntakeMotor).setPower(0);
        hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);

        intake.extension.runWithoutEncoder();
        outtake.extension.setPosition(40);
        intake.extension.extend(-1);
        timer.reset();

        while (opMode.opModeIsActive()
                &&
                intake.extension.getPosition() > 10
                &&
                !intake.extension.constrained()) {
            hardware.bulk.clearCache(Hubs.ALL);
            outtake.extension.update();
        }

        hardware.servos.get(IntakeWrist).setPosition(intake.wristTransfer);
        intake.extension.extend(0);
        intake.extension.runToPosition();
        intake.extension.disable();

        // claw passthrough
        hardware.servos.get(OuttakeLeftPivot).setPosition(outtake.armTransfer);
        hardware.servos.get(OuttakeRightPivot).setPosition(outtake.armTransfer);

        try { Thread.sleep(200); } catch (InterruptedException e) {}

        outtake.openClaw(false);
        hardware.servos.get(IntakeLocker).setPosition(intake.lockerOpen);

        try { Thread.sleep(170); } catch (InterruptedException e) {}
    }




    // fail safes
    public void whenTransferFails() {
        outtake.openClaw(true);
        outtake.setAction(OuttakeAction.PRE_TRANSFER);

        hardware.servos.get(IntakeLocker).setPosition(intake.lockerClosed);
        hardware.servos.get(IntakeWrist).setPosition(intake.wristUp);

        try { Thread.sleep(190); } catch (InterruptedException e) {}

        disableOuttake();
        intake.setAction(IntakeAction.COLLECT);

    }


    private void disableOuttake() {
        hardware.servos.get(OuttakeClaw).getController().pwmDisable();
        outtake.extension.disable();

        intake.manual = true;
        outtake.manual = false;
        SystemConstants.updateOuttake = false;
    }

    private void disableIntake() {
        hardware.servos.get(IntakeLocker).getController().pwmDisable();
        intake.extension.disable();

        intake.manual = false;
        outtake.manual = true;
        SystemConstants.updateOuttake = true;
    }



    // scoring
    public void specimens() {
        // collect from the field wall

        outtake.openClaw(false);
        try { Thread.sleep(300); } catch (InterruptedException e) {}

        outtake.waitReached(LiftAction.HIGH_RUNG);
        outtake.setAction(OuttakeAction.SCORE_SPECIMENS);
    }

    public void samplesHigh() {
        hardware.motors.get(IntakeMotor).setPower(-0.8);
        try { Thread.sleep(300); } catch (InterruptedException e) {}

        waitReachedOuttake = 200;

        outtake.waitReached(LiftAction.HIGH_BASKET);
        outtake.setAction(OuttakeAction.SCORE_SAMPLES);

        waitReachedOuttake = 6;


        hardware.motors.get(IntakeMotor).setPower(0);
    }

    public void score() {
        switch (outtake.getAction()) {
            case SCORE_SAMPLES: {
                outtake.moveArm(0.78);
                hardware.servos.get(OuttakeWrist).setPosition(outtake.wristTransfer);
                try { Thread.sleep(120); } catch (InterruptedException e) {}
                outtake.openClaw(true);

                try { Thread.sleep(200); } catch (InterruptedException e) {}


                outtakeToTransfer();
            } break;

            case SCORE_SPECIMENS: {
                outtake.moveArm(0.6);

                try { Thread.sleep(400); } catch (InterruptedException e) {}

                outtakeToTransfer();
            } break;
        }

        successfulCatch = false;
    }


}
