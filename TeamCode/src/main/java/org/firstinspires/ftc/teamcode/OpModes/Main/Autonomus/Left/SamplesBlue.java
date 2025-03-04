package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Left;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.basketPose;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extendoMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.successfulCatch;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeWrist;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Autonomous(group = "aa_main", preselectTeleOp = "✨ 🅻🍓🆅🅴 ✨")
public class SamplesBlue extends ExoMode {
    private Machine robot;
    private AutoDrive auto;

    private final Pose
            firstSample = new Pose(18.2, 13, Math.toRadians(-12)),
            secondSample = new Pose(18.2, 20, Math.toRadians(2)),
            thirdSample = new Pose(16, 21.1, Math.toRadians(24));

    private final double
            leftUltraInch = 4.5,
            rightUltraInch = 4.5;

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .setAutoOnBlue(true)
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
        firstSample();
        secondSample();
        thirdSample();
        //fourthSample();


        auto.end();
    }



    private void preload() {
        auto
                // init scoring system
                .moveSystem(() -> {
                    robot.system.outtake.openClaw(false);
                    robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                })

                // move the outtake slightly just to be able to lift the arm and move on with your life
                .moveSystem(() -> {
                    SystemConstants.waitReachedOuttake = 700;
                    robot.system.samplesHigh();
                })

                // 1st basket drive. Aligning with odometry because ultrasonics suck
                .driveTo(basketPose)
                .waitDrive()
                .validateBasket(leftUltraInch, rightUltraInch)
                .driveTo(basketPose)
                .waitDrive()

                // after exiting alignment loop, wait for the outtake to extend, if necessary
                .waitAction(() -> robot.system.outtake.extension.reached(29))
                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeWrist).setPosition(robot.system.intake.wristUp);
                    robot.system.score();
                });
    }

    private void firstSample() {
        auto
                // drive to the first sample position + lower the intake at the same time to save time
                .moveSystem(() -> robot.system.lowerIntakeAndStart())
                .driveTo(firstSample)

                .waitMs(200)
                .moveSystem(() -> { robot.system.intake.extension.setPosition(extendoMAX);})

                .waitDrive()
                // wait for the sample to enter the intake and be ready for transfer
                //     if the sample doesn't get inside in 1 second, abandon it and continue with the transfer sequence
                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.updateAutoTransferAuto(),
                        1200,
                        () -> {
                            // spit in the case of a partial intake that did jam
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(150); } catch (InterruptedException e) {}
                            robot.system.autoTransferSequence();
                        })


                //..................................................................................
                .moveSystem(() -> {
                    robot.system.samplesHigh();
                })

                .driveTo(basketPose)
                .waitDrive()
                .validateBasket(leftUltraInch, rightUltraInch)
                .driveTo(basketPose)
                .waitDrive()

                .waitAction(() -> robot.system.outtake.extension.reached(29))
                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeWrist).setPosition(robot.system.intake.wristUp);
                    robot.system.score();
                });
    }

    private void secondSample() {
        auto
                .moveSystem(() -> robot.system.lowerIntakeAndStart())
                .driveTo(secondSample)

                .waitMs(200)
                .moveSystem(() -> robot.system.intake.extension.setPosition(extendoMAX))

                .waitDrive()
                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.updateAutoTransferAuto(),
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(150); } catch (InterruptedException e) {}
                            robot.system.autoTransferSequence();
                        })

                // same outtake process
                .moveSystem(() -> {
                    robot.system.samplesHigh();
                })

                .driveTo(basketPose)
                .waitDrive()
                .validateBasket(leftUltraInch, rightUltraInch)
                .driveTo(basketPose)
                .waitDrive()

                .waitAction(() -> robot.system.outtake.extension.reached(29))
                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeWrist).setPosition(robot.system.intake.wristUp);
                    robot.system.score();
                });
    }

    private void thirdSample() {
        auto
                .moveSystem(() -> robot.system.lowerIntakeAndStart())
                .driveTo(thirdSample)

                .waitMs(200)
                .moveSystem(() -> robot.system.intake.extension.setPosition(extendoMAX))

                .waitDrive()
                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> robot.system.updateAutoTransferAuto(),
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(20); } catch (InterruptedException e) {}
                            robot.system.autoTransferSequence();
                        })


                .moveSystem(() -> {
                    robot.system.samplesHigh();
                    SystemConstants.waitReachedOuttake = 10;
                })

                .driveTo(basketPose)
                .waitDrive()
                .validateBasket(leftUltraInch, rightUltraInch)
                .driveTo(basketPose)
                .waitDrive()

                .waitAction(() -> robot.system.outtake.extension.reached(29))
                .moveSystem(() -> {
                    robot.hardware.servos.get(IntakeWrist).setPosition(robot.system.intake.wristUp);
                    robot.system.score();
                });
    }

    private void fourthSample() {
        auto
                .driveTo(new Pose(50, 0, Math.toRadians(90)))
                .waitMs(1200)
                .driveTo(new Pose(58, 20, Math.toRadians(90)))
                .waitDrive()

                .moveSystem(() -> { robot.system.intake.extension.setPosition(90);})
                .moveSystem(() -> {robot.system.lowerIntakeAndStart();
                    robot.system.intake.extension.setPosition(extendoMAX);})


                .waitActionTimeFailSafe(() -> successfulCatch,
                        () -> {
                            if (successfulCatch) {
                                auto.driveTo(basketPose);
                                robot.system.autoTransferSequence();
                            }
                        },
                        1200,
                        () -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.MOTOR_SPIT);
                            try { Thread.sleep(20); } catch (InterruptedException e) {}
                            robot.system.transferSequence();
                        })

                .moveSystem(() -> {
                    SystemConstants.waitReachedOuttake = 700;
                    robot.system.samplesHigh();
                    SystemConstants.waitReachedOuttake = 10;
                })

                .waitDrive()
                .validateBasket(leftUltraInch, rightUltraInch)
                .driveTo(basketPose)
                .waitDrive()

                .moveSystem(() -> robot.system.score());
    }


    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {}
}
