package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeDistance;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeExtension;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.TwoMotorLift;

public class Outtake implements Enums.OuttakeEnums {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static OuttakeAction previousAction, currentAction;
    private static ArmAction previousArmAction;
    public final TwoMotorLift extension;

    private static final double
            clawOpen = 0.65,
            clawClosed = 0.9;

    private static final double
            armTransfer = 0,
            armScore = 0.77;

    private static final double
            extendoTransfer = 0.4,
            extendoPreScore = 0.82,
            extendoScore = 0.4;


    private static final double
            wristTransfer = 0.72,
            wristScore = 0.40;

    private static final double // mm
            inverseMIN = 0,
            inverseMAX = 160; // 160 mm is the mechanical limit (of the extendo)



    private double kinematicPose = 0;
    private final double j1GearRatio = 24.0 / 40;

    /**Inverse Kinematics constants*/
    private final double alpha = Math.toRadians(117);
    private final double r = 214;

    private final double  a = 1,
                          b = 2 * r;



    public Outtake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap, opMode.telemetry);
        hardware.servos.get(OuttakeRightPivot).setDirection(Servo.Direction.REVERSE);

        this.opMode = opMode;
        this.extension = new TwoMotorLift(opMode, LeftOuttakeMotor, RightOuttakeMotor);

        for (LiftAction action : LiftAction.values()) {
            extension.add(action.name(), action.ticks);
        }

        extension.addLimit(outtakeMAX)
                .addCurrentAlert(1)
                .addGearRatios(10.0 / 8, 8.0 / 8)
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

    public void setArmAction(ArmAction action) {
        switch (action) {
            case PRE_SCORE: {
                if (hardware.servos.get(OuttakeRightPivot).getPosition() != armScore) {
                    hardware.servos.get(OuttakeLeftPivot).setPosition(armScore);
                    hardware.servos.get(OuttakeRightPivot).setPosition(armScore);
                }
                hardware.servos.get(OuttakeExtension).setPosition(extendoTransfer);

                if (hardware.servos.get(OuttakeClaw).getPosition() != clawClosed)
                    hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
                hardware.servos.get(OuttakeWrist).setPosition(wristScore);

                try { Thread.sleep(400); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeExtension).setPosition(extendoPreScore);

            } break;

            case SCORE_SPECIMENS: {
                hardware.servos.get(OuttakeExtension).setPosition(extendoScore);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);

            } break;

            case SCORE_SAMPLES: {
                hardware.servos.get(OuttakeWrist).setPosition(wristScore + 0.7);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);

            } break;

            case PRE_TRANSFER: {
                hardware.servos.get(OuttakeLeftPivot).setPosition(armTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armTransfer);

                hardware.servos.get(OuttakeExtension).setPosition(extendoTransfer);
                hardware.servos.get(OuttakeWrist).setPosition(wristTransfer);

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);
            } break;

            case TRANSFER: {
                double distance = hardware.distance.get(OuttakeDistance).getDistance(DistanceUnit.MM);
                inverseKinematics(distance);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);

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
                setLiftAction(LiftAction.TRANSFER);
                setArmAction(ArmAction.PRE_TRANSFER);
            } break;

            case TRANSFER: {
                setArmAction(ArmAction.TRANSFER);

                try { Thread.sleep(100); } catch (InterruptedException e) {}
                extension.disable();
            } break;

            case SCORE: {
                setArmAction(SystemConstants.outtakeScore);
            } break;

            case PRE_SCORE: {
                switch (SystemConstants.outtakeScore) {
                    case SCORE_SAMPLES: { action = OuttakeAction.SCORE_HIGH_BASKET; } break;
                    case SCORE_SPECIMENS: { action = OuttakeAction.SCORE_HIGH_RUNG; } break;
                }
            } ;


            case SCORE_HIGH_RUNG: {
                setArmAction(ArmAction.PRE_SCORE);
                setLiftAction(LiftAction.HIGH_RUNG);
            } break;

            case SCORE_HIGH_BASKET: {
                setArmAction(ArmAction.PRE_SCORE);
                setLiftAction(LiftAction.HIGH_BASKET);
            }

        }

        previousAction = currentAction;
        currentAction = action;
    }


    public void extend(double power, boolean noLimit) { extension.extend(power, noLimit);}

    public void extend(double power) { extension.extend(power); }


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

        double dl = (-b + Math.sqrt(delta)) / 2;
        double R = dl + r;

        double dtheta1 = Math.acos((target * target - r * r - R * R) / (-2 * r * R));

        double theta1 = Math.toDegrees(dtheta1) * armScore / 153;
        double theta2 = extendoTransfer + dl * (extendoPreScore - extendoTransfer) / 106;
        double theta3 = wristTransfer - j1GearRatio * theta1;

        moveKinematics(theta1, theta2, theta3);
        kinematicPose = target;
    }

    private void moveKinematics(double theta1, double theta2, double theta3) {
        hardware.servos.get(OuttakeLeftPivot).setPosition(theta1);
        hardware.servos.get(OuttakeRightPivot).setPosition(theta1);

        hardware.servos.get(OuttakeExtension).setPosition(Math.max(theta2 - 0.09, extendoTransfer)); //inconsistency in arm linearity
        hardware.servos.get(OuttakeWrist).setPosition(theta3);
    }



    public OuttakeAction getAction() { return currentAction; }

    public OuttakeAction getPreviousAction() { return previousAction; }

    public boolean hasJustChangedTo(OuttakeAction action) { return action == currentAction && action != previousAction; }


    public void update() { extension.update(); }


    public double getKinematicPose() { return kinematicPose; }
}
