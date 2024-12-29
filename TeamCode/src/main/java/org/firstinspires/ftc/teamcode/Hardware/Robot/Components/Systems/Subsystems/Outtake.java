package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.driveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastTurn;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowTurn;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.distance_sensorToClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extensionMultiplier;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeDistance;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeExtension;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRotation;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.TwoMotorLift;

import java.util.concurrent.TimeUnit;

public class Outtake implements Enums.OuttakeEnums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static OuttakeAction previousAction, currentAction;
    private static ArmAction previousArmAction;
    public final TwoMotorLift extension;

    private static final double
            clawTransferOpen = 0.4,
            clawFullyOpen = 0.43,
            clawClosed = 0.6;

    private static final double
            armTransfer = 0.01,
            armScore = 0.85;

    private static final double
            rotationTransfer = 0.00,
            rotationSpecimen = 0.72;

    private static final double
            extendoTransfer = 0.32,
            extendoScoreSamples = 0.85,
            extendoScoreSpecimens = 0.3;


    private static final double
            wristTransfer = 0.875,
            wristScoreSpecimens = 0.72,
            wristScoreSamples = 0.80;

    private static final double // mm
            inverseMIN = 0,
            inverseMAX = 160; // 160 mm is the mechanical limit (of the extendo)

    private final double p = 0.08, d = 0, f = 0.025;

    private double kinematicPose = 0;
    private final double j1GearRatio = 24.0 / 40;

    /**Inverse Kinematics constants*/
    private final double alpha = Math.toRadians(117);
    private final double r = 214;

    private final double  a = 1,
                          b = 2 * r;



    public Outtake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        hardware.servos.get(OuttakeRightPivot).setDirection(Servo.Direction.REVERSE);

        this.opMode = opMode;
        this.extension = new TwoMotorLift(opMode, LeftOuttakeMotor, RightOuttakeMotor);

        for (LiftAction action : LiftAction.values()) {
            extension.add(action.name(), action.ticks);
        }

        extension.addLimit(outtakeMAX)
                .addCurrentAlert(1)
                .addGearRatios(10.0 / 8, 8.0 / 8)
                .setPID(p, 0, d)
                .reverseRight();
    }



    public void openClaw(boolean open) {
        hardware.servos.get(OuttakeClaw).setPosition((open) ? clawFullyOpen : clawClosed);
    }

    public void toggleClaw() {
        if (hardware.servos.get(OuttakeClaw).getPosition() == clawFullyOpen)
            openClaw(false);
        else openClaw(true);
    }

    public void setArmAction(ArmAction action) {
        switch (action) {
            case DISABLE: {
                hardware.servos.get(OuttakeLeftPivot).getController().close();
                hardware.servos.get(OuttakeRightPivot).getController().close();

                hardware.servos.get(OuttakeClaw).getController().close();
                hardware.servos.get(OuttakeWrist).getController().close();
                hardware.servos.get(OuttakeRotation).getController().close();
                hardware.servos.get(OuttakeExtension).getController().close();
            } break;

            case PRE_SCORE: {
                if (hardware.servos.get(OuttakeClaw).getPosition() != clawClosed) {
                    hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
                    try { Thread.sleep(200); } catch (InterruptedException e) {}
                }

                hardware.servos.get(OuttakeLeftPivot).setPosition(armScore);
                hardware.servos.get(OuttakeRightPivot).setPosition(armScore);


            } break;

            case SCORE_SPECIMENS: {
                setLiftAction(LiftAction.TRANSFER);
                ElapsedTime time = new ElapsedTime();

                while (opMode.opModeIsActive() && time.time(TimeUnit.MILLISECONDS) < 400) {


                    extension.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);
                }

                hardware.servos.get(OuttakeClaw).setPosition(clawFullyOpen);
                setLiftAction(LiftAction.HIGH_RUNG);

                time.reset();
                while (opMode.opModeIsActive() && time.time(TimeUnit.MILLISECONDS) < 100) {
                    extension.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);
                }

                setLiftAction(LiftAction.TRANSFER);

                time.reset();
                while (opMode.opModeIsActive() && time.time(TimeUnit.MILLISECONDS) < 200) {
                    extension.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);
                }


            } break;

            case SCORE_SAMPLES: {
                hardware.servos.get(OuttakeClaw).setPosition(clawFullyOpen - 0.18);
            } break;

            case PRE_TRANSFER: {
                hardware.servos.get(OuttakeRotation).setPosition(rotationTransfer);
                hardware.servos.get(OuttakeClaw).setPosition(clawTransferOpen);
                hardware.servos.get(OuttakeExtension).setPosition(extendoTransfer);
                hardware.servos.get(OuttakeLeftPivot).setPosition(armTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armTransfer);

                try { Thread.sleep(190); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeRotation).getController().close();

                inverseKinematics(30);

            } break;

            case TRANSFER: {
                hardware.servos.get(OuttakeRotation).setPosition(rotationTransfer);
                hardware.servos.get(OuttakeClaw).setPosition(clawTransferOpen);
                hardware.servos.get(OuttakeExtension).setPosition(extendoTransfer);

                try { Thread.sleep(100); } catch (InterruptedException e) {} // debugging (default 100)

                double distance_sensorToSample = hardware.distance.get(OuttakeDistance).getDistance(DistanceUnit.MM);
                inverseKinematics(distance_sensorToSample - distance_sensorToClaw + 30);

                hardware.telemetry.addData("Transfer Distance: ", distance_sensorToSample);
                hardware.telemetry.update();

                try { Thread.sleep((long) (160 * distance_sensorToSample / 70.0)); } catch (InterruptedException e) {} // debugging (default 100)

                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);

                try { Thread.sleep(160); } catch (InterruptedException e) {}

            } break;

        }

        previousArmAction = action;
    }



    private void setLiftAction(LiftAction action) { extension.setPosition(action.name()); }

    public void setAction(OuttakeAction action) {
        switch (action) {

            case HANG: {
                setLiftAction(LiftAction.FULL);
                setArmAction(ArmAction.PRE_SCORE);
            } break;

            case PRE_TRANSFER: {
                driveSensitivity = fastTurn;

                setLiftAction(LiftAction.TRANSFER);

                if (previousArmAction != ArmAction.PRE_TRANSFER)
                    setArmAction(ArmAction.PRE_TRANSFER);

                while (opMode.opModeIsActive() && !extension.reached(20)) {
                    extension.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);
                }

                extension.disable();
            } break;

            case TRANSFER: {
                setArmAction(ArmAction.TRANSFER);

                try { Thread.sleep(100); } catch (InterruptedException e) {}
            } break;

            case SCORE: {
                setArmAction(SystemConstants.outtakeScore);
            } break;

            case PRE_SCORE: {
                driveSensitivity = slowTurn;

                switch (SystemConstants.outtakeScore) {
                    case SCORE_SAMPLES: { setAction(OuttakeAction.SCORE_HIGH_BASKET); } break;
                    case SCORE_SPECIMENS: { setAction(OuttakeAction.SCORE_HIGH_RUNG); } break;
                }
            } break;


            case SCORE_HIGH_RUNG: {
                setLiftAction(LiftAction.HIGH_RUNG);
                setArmAction(ArmAction.PRE_SCORE);

                hardware.servos.get(OuttakeRotation).setPosition(rotationSpecimen);

                hardware.servos.get(OuttakeWrist).setPosition(wristScoreSpecimens);
                hardware.servos.get(OuttakeExtension).setPosition(extendoScoreSpecimens);
                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
            } break;

            case SCORE_HIGH_BASKET: {
                setLiftAction(LiftAction.HIGH_BASKET);

                hardware.servos.get(OuttakeRotation).getController().close();
                setArmAction(ArmAction.PRE_SCORE);

                hardware.servos.get(OuttakeWrist).setPosition(wristScoreSamples);
                hardware.servos.get(OuttakeExtension).setPosition(extendoScoreSamples);
                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
            } break;

        }

        previousAction = currentAction;
        currentAction = action;
    }


    public void extend(double power, boolean noLimit) { extension.extend(power, noLimit);}

    public void extend(double power) {
        extension.extend(power);
    }


    /** Uses the TRIGONOMETRY CIRCLE with DEGREES (°) as reference system
     *
     * ALL JOINTS have their own designed coordinate systems with
     *      the convention from above
     *
     * @param target is the input scalar of the straight line between
     *               the claw and the sample. Measured in mm (distance)
     */
    public void inverseKinematics(double target) {
        target = Math.max(inverseMIN, Math.min(inverseMAX, target));

        double c = 2 * r * target * Math.cos(alpha) - target * target;
        double delta = b * b - 4 * a * c;

        double dl = (-b + Math.sqrt(delta)) / 2 * extensionMultiplier;
        double R = dl + r;

        double dtheta1 = Math.acos((target * target - r * r - R * R) / (-2 * r * R));

        double theta1 = armTransfer + Math.toDegrees(dtheta1) * armScore / 153;
        double theta2 = extendoTransfer + dl * (extendoScoreSamples - extendoTransfer) / 106;
        double theta3 = wristTransfer - j1GearRatio * theta1;

        moveKinematics(theta1, theta2, theta3);
        kinematicPose = target;
    }

    private void moveKinematics(double theta1, double theta2, double theta3) {
        hardware.servos.get(OuttakeLeftPivot).setPosition(theta1);
        hardware.servos.get(OuttakeRightPivot).setPosition(theta1);

        hardware.servos.get(OuttakeExtension).setPosition(theta2);
        hardware.servos.get(OuttakeWrist).setPosition(theta3);
    }



    public OuttakeAction getAction() { return currentAction; }

    public OuttakeAction getPreviousAction() { return previousAction; }

    public ArmAction getPreviousArmAction() { return previousArmAction; }

    public double getKinematicPose() { return kinematicPose; }



    public boolean hasJustChangedTo(OuttakeAction action) { return action == currentAction && action != previousAction; }

}
