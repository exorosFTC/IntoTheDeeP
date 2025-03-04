package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Right;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.successfulCatch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeLocker;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeTurret;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Autonomous(group = "aa_main", preselectTeleOp = "✨ 🅻🍓🆅🅴 ✨")
public class SpecimenRed extends ExoMode {
    private Machine robot;
    private AutoDrive auto;

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .setAutoOnBlue(false)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false)
                        .setUsingAcceleration(false)
                        .setUsingExponentialInput(false))
                .construct(this);

        robot.system.intake.extension.resetEncoders();
        robot.system.intake.extension.runToPosition();

        robot.system.outtake.extension.resetEncoders();

        SystemConstants.updateOuttake = true;
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, robot, new Pose());

        preload();
        driveSpecimens();
        scoreThem();

        auto.end();
    }

    private void preload() {
        auto
                // init scoring system
                .moveSystem(() -> {
                    robot.system.outtake.openClaw(false);
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                })
                .driveTo(new Pose(25  , 7.1, Math.toRadians(0)))

                .moveSystem(() -> {
                    robot.system.outtake.waitReached(Enums.OuttakeEnums.LiftAction.HIGH_RUNG);
                    robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SPECIMENS);
                })

                .waitDrive()

                .driveTillScoreSpecimen();

    }

    private void driveSpecimens() {
        auto
                .driveTo(new Pose(10, -10, Math.toRadians(-15)))
                .waitMs(350)
                .driveTo(new Pose(17.2, -28, Math.toRadians(-15)))
                .moveSystem(() -> robot.system.lowerIntakeAndStart())

                .moveSystem(() -> robot.system.intake.extension.setPosition(extendoMAX))

                .moveSystem(() -> robot.hardware.servos.get(IntakeTurret).setPosition(0.29))

                .waitDrive()

                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.waitCatch(),
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(20); } catch (InterruptedException e) {}
                        })
                .moveSystem(() -> {
                    robot.system.liftIntake();
                    robot.system.intake.extension.setPosition(200);
                })
                .waitMs(130)

                .driveTo(new Pose(19, -30, Math.toRadians(-150)))
                .waitDrive()
                .moveSystem(() -> {
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);

                    try { Thread.sleep(200); } catch (InterruptedException e) {}

                    robot.hardware.servos.get(IntakeLocker).setPosition(robot.system.intake.lockerOpen);
                    robot.hardware.motors.get(IntakeMotor).setPower(-1);

                    successfulCatch = false;

                    try { Thread.sleep(200); } catch (InterruptedException e) {}
                })



                .moveSystem(() -> {
                    robot.system.liftIntake();
                })

                .driveTo(new Pose(20, -28.9, Math.toRadians(-45)))

                .waitDrive()

                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeTurret).setPosition(robot.system.intake.turretCollect);
                    robot.system.lowerIntakeAndStart();
                })

                .waitDrive()

                .moveSystem(() -> {
                    robot.system.intake.extension.setPosition(extendoMAX);
                })







                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.waitCatch(),
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(20); } catch (InterruptedException e) {}
                        })
                .moveSystem(() -> {
                    robot.system.liftIntake();
                    robot.system.intake.extension.setPosition(200);
                })
                .waitMs(130)

                .driveTo(new Pose(19, -30, Math.toRadians(-170)))
                .waitDrive()
                .moveSystem(() -> {
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);

                    try { Thread.sleep(200); } catch (InterruptedException e) {}

                    robot.hardware.servos.get(IntakeLocker).setPosition(robot.system.intake.lockerOpen);
                    robot.hardware.motors.get(IntakeMotor).setPower(-1);

                    successfulCatch = false;

                    try { Thread.sleep(200); } catch (InterruptedException e) {}
                })

                .moveSystem(() -> {
                    robot.system.liftIntake();
                })










                .driveTo(new Pose(15, -37.7, Math.toRadians(-42)))

                .waitDrive()

                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeTurret).setPosition(robot.system.intake.turretCollect);
                    robot.system.lowerIntakeAndStart();
                })

                .waitDrive()

                .moveSystem(() -> {
                    robot.system.intake.extension.setPosition(extendoMAX);
                })







                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.waitCatch(),
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(20); } catch (InterruptedException e) {}
                        })
                .moveSystem(() -> {
                    robot.system.liftIntake();
                    robot.system.intake.extension.setPosition(200);
                })
                .waitMs(130)

                .driveTo(new Pose(20, -31, Math.toRadians(-170)))
                .waitDrive()
                .moveSystem(() -> {
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);

                    try { Thread.sleep(200); } catch (InterruptedException e) {}

                    robot.hardware.servos.get(IntakeLocker).setPosition(robot.system.intake.lockerOpen);
                    robot.hardware.motors.get(IntakeMotor).setPower(-1);

                    successfulCatch = false;

                    try { Thread.sleep(200); } catch (InterruptedException e) {}
                })

                .moveSystem(() -> {
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                });

    }

    private void scoreThem() {
        auto
                .driveTo(new Pose(8, -29, Math.toRadians(0)))
                .moveSystem(() -> {
                    robot.hardware.servos.get(OuttakeWrist).setPosition(robot.system.outtake.wristCollectSpecimens);
                    robot.system.collectSpecimensIntakeAccess();
                })
                .waitDrive()
                .slowlyDriveToSpecsDistance(4)
                .moveSystem(() -> {
                    robot.system.outtake.openClaw(false);
                })
                .waitMs(200)

                .driveTo(new Pose(25  , 6, Math.toRadians(0)))

                .moveSystem(() -> {
                    robot.system.outtake.waitReached(Enums.OuttakeEnums.LiftAction.HIGH_RUNG);
                    robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SPECIMENS);
                })

                .waitDrive()
                .waitMs(200)
                .driveTillScoreSpecimen()









                .driveTo(new Pose(15, 5, Math.toRadians(0)))
                .moveSystem(() -> {
                    robot.system.intake.extendUntilZero();
                })
                .driveTo(new Pose(8, -26, Math.toRadians(0)))

                .moveSystem(() -> {
                    robot.hardware.servos.get(OuttakeWrist).setPosition(robot.system.outtake.wristCollectSpecimens);
                    robot.system.collectSpecimensIntakeAccess();

                })
                .waitDrive()
                .waitMs(200)
                .slowlyDriveToSpecsDistance(4)
                .moveSystem(() -> {
                    robot.system.outtake.openClaw(false);
                })
                .waitMs(200)

                .driveTo(new Pose(25  , 4, Math.toRadians(0)))

                .moveSystem(() -> {
                    robot.system.outtake.waitReached(Enums.OuttakeEnums.LiftAction.HIGH_RUNG);
                    robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SPECIMENS);
                })

                .waitDrive()
                .moveSystem(() -> {
                    robot.system.intake.extendUntilZero();
                })
                .driveTillScoreSpecimen()










                .driveTo(new Pose(11, 5, Math.toRadians(0)))
                .moveSystem(() -> {
                    robot.system.intake.extendUntilZero();
                })
                .driveTo(new Pose(8, -24, Math.toRadians(0)))


                .moveSystem(() -> {
                    robot.hardware.servos.get(OuttakeWrist).setPosition(robot.system.outtake.wristCollectSpecimens);
                    robot.system.collectSpecimensIntakeAccess();
                })

                .waitDrive()
                .waitMs(200)

                .slowlyDriveToSpecsDistance(4)
                .moveSystem(() -> {
                    robot.system.outtake.openClaw(false);
                })
                .waitMs(200)

                .driveTo(new Pose(29  , 4, Math.toRadians(0)))

                .moveSystem(() -> {
                    robot.system.outtake.waitReached(Enums.OuttakeEnums.LiftAction.HIGH_RUNG);
                    robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE_SPECIMENS);
                })

                .waitDrive()
                .driveTillScoreSpecimen()

                .driveTo(new Pose(11, 5, Math.toRadians(0)))
                .moveSystem(() -> {
                    robot.system.intake.extendUntilZero();
                })
                .driveTo(new Pose(8, -26, Math.toRadians(0)))

                .waitDrive()
                .moveSystem(() -> robot.system.intake.extendUntilZero());


    }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {}
}
