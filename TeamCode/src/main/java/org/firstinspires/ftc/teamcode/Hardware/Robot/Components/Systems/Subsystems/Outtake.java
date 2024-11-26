package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.inOuttakeThreshold;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeClaw;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeExtension;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.cameraConfigurationName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.TwoMotorLift;

public class Outtake implements Enums.Outtake {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static OuttakeAction lastAction;
    private static ArmAction lastArmAction;
    public final TwoMotorLift extension;

    private static final double
            clawOpen = 0.65,
            clawClosed = 0.9;

    private static final double
            armTransfer = 0,
            armScore = 0.75;

    private static final double // [0.05, 0.50] --- retracted / full extension
            extendoTransfer = 0.5,
            extendoPreScore = 0.82,
            extendoScore = 0.47;


    private static final double
            wristTransfer = 0.7,
            wristScore = 0.55;

    private static final double // mm
            inverseMIN = 0,
            inverseMAX = 60;



    private double kinematicPose = 0;
    private final double j1GearRatio = 24 / 40;
    private final double maxExtension = 0.5;

    /** assumes that pose 0 (from the interval [0, 1]
     *      is this value (in DEGREES) irl*/
    private final double j1_start = 298;
    private final double j3_start = 108;

    /** linear extension starting length (in MILIMETERS)
     *
     *  length of the rail + 14 mm of extension*/
    private final double j2_start = 200 + 14;



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
                .addGearRatios(10 / 8, 8 / 8)
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

                if (hardware.servos.get(OuttakeClaw).getPosition() != clawClosed)
                    hardware.servos.get(OuttakeClaw).setPosition(clawClosed);
                hardware.servos.get(OuttakeWrist).setPosition(wristScore);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeExtension).setPosition(extendoPreScore);

            } break;

            case SCORE: {
                if (lastArmAction != ArmAction.PRE_SCORE)
                    break;

                hardware.servos.get(OuttakeExtension).setPosition(extendoScore);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                setArmAction(ArmAction.PRE_TRANSFER);
            } break;

            case PRE_TRANSFER: {
                hardware.servos.get(OuttakeWrist).setPosition(wristTransfer);

                hardware.servos.get(OuttakeExtension).setPosition(extendoTransfer);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeLeftPivot).setPosition(armTransfer);
                hardware.servos.get(OuttakeRightPivot).setPosition(armTransfer);


                try { Thread.sleep(100); } catch (InterruptedException e) {}

                hardware.servos.get(OuttakeClaw).setPosition(clawOpen);
                kinematicPose = 0;
            } break;

            case TRANSFER: {
                if (lastArmAction != ArmAction.PRE_TRANSFER)
                    break;

                double distance = hardware.color.get(OuttakeColor).getDistance(DistanceUnit.MM);
                inverseKinematics(kinematicPose + distance);

                hardware.servos.get(OuttakeClaw).setPosition(clawClosed);

            } break;

        }

        lastArmAction = action;
    }



    private void setLiftAction(LiftAction action) { extension.setPosition(action.name()); }

    public void setAction(OuttakeAction action) {
        switch (action) {
            case INIT: {
                setArmAction(ArmAction.PRE_TRANSFER);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                setLiftAction(LiftAction.ZERO);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                extension.disable();
            } break;

            case HANG: {
                setLiftAction(LiftAction.FULL);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                setArmAction(ArmAction.PRE_SCORE);
            } break;

            case TRANSFER: {
                setArmAction(ArmAction.TRANSFER);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                setLiftAction(LiftAction.ZERO);

                try { Thread.sleep(100); } catch (InterruptedException e) {}

                extension.disable();
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

        double alpha = Math.toRadians(117);
        double r = 214;

        double  a = 1,
                b = 2 * r,
                c = 2 * r * target * Math.cos(alpha) - target * target;
        double delta = b * b - 4 * a * c;

        hardware.telemetry.addData("delta: ", delta);

        double dl = (-b + Math.sqrt(delta)) / 2;

        hardware.telemetry.addData("dl: ", dl);

        double R = dl + r;

        double dtheta1 = Math.PI - Math.acos((target * target - r * r - R * R) / (2 * r * R));

        hardware.telemetry.addData("dtheta1", dtheta1);

        double theta1 = Math.toDegrees(dtheta1) * armScore / 153;
        double theta2 = extendoTransfer + dl * (extendoPreScore - extendoTransfer) / 106;
        double theta3 = wristTransfer - j1GearRatio * theta1;

        kinematicPose = target;

        hardware.telemetry.addData("Joint 1 Position: ", theta1);
        hardware.telemetry.addData("Joint 2 Position: ", theta2);
        hardware.telemetry.addData("Joint 3 Position: ", theta3);
        hardware.telemetry.update();

        //hardware.servos.get(OuttakeLeftPivot).setPosition(theta1);
        //hardware.servos.get(OuttakeRightPivot).setPosition(theta1);
        //hardware.servos.get(OuttakeExtension).setPosition(theta2);
        //hardware.servos.get(OuttakeWrist).setPosition(theta3);
    }


    public OuttakeAction getAction() { return lastAction; }
}
