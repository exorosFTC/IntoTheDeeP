package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Autonomous(group = "aa_main", preselectTeleOp = "🍓")
public class RedLeft extends ExoMode {
    private Machine robot;

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
    }

    @Override
    protected void WhenStarted() {
        AutoDrive auto = new AutoDrive(this, robot)
                        // score the pre-loaded specimen
                        .moveSystem(() -> robot.system.outtake.openClaw(false))
                .driveTo(new Pose(24, 5, Math.toRadians(180)))
                        .moveSystem(() -> {
                            SystemConstants.outtakeScore = Enums.OuttakeEnums.ArmAction.SCORE_SPECIMENS;
                            robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_SCORE);
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                            robot.system.intake.extension.setPosition(Enums.IntakeEnums.IntakePosition.ZERO.name());

                        })
                .waitDrive()
                        .moveSystem(() -> robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE))

                // move to sample pickup position
                .driveTo(new Pose(17, -47.8, Math.toRadians(19)))
                .waitMs(800)
                        .moveSystem(() -> {
                            robot.system.intake.extension.setPosition(433);
                            robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_TRANSFER);

                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.PRE_COLLECT);                            robot.hardware.servos.get(IntakeRotation).setPosition(0);
                            robot.hardware.servos.get(IntakeRotation).setPosition(0);

                        })
                .waitDrive()
                        .moveSystem(() -> {
                            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.COLLECT);
                            robot.system.transfer();

                            SystemConstants.outtakeScore = Enums.OuttakeEnums.ArmAction.SCORE_SAMPLES;
                            robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_SCORE);
                        })
                .driveTo(new Pose(8, -47.5, Math.toRadians(35)))
                .waitMs(900)
                        .moveSystem(() -> robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE))
                .end();
    }

    @Override
    protected void InitializeThreads() {

    }

    @Override
    protected void Loop() {

    }
}
