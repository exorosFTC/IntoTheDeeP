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
public class RedRight extends ExoMode {
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
                .driveTo(new Pose(23, -5, Math.toRadians(180)))
                    .moveSystem(() -> {
                        SystemConstants.outtakeScore = Enums.OuttakeEnums.ArmAction.SCORE_SPECIMENS;
                        robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_SCORE);
                        robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
                        robot.system.intake.extension.setPosition(Enums.IntakeEnums.IntakePosition.ZERO.name());

                    })
                .waitDrive()
                    .moveSystem(() -> robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE))

                // move to the hp station
                .driveTo(new Pose(16, 49.2, Math.toRadians(0)))
                .waitMs(900)
                    .moveSystem(() -> {
                        robot.system.intake.extension.setPosition(100);
                        robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.PRE_COLLECT);
                        robot.hardware.servos.get(IntakeRotation).setPosition(0);

                        robot.system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.PRE_TRANSFER);
                    })

                //go for the first sample
                .driveTo(new Pose(10, 30, Math.toRadians(0)))
                .waitDrive()
                .driveTo(new Pose(50, 37, Math.toRadians(0)))
                .waitDrive()
                .driveTo(new Pose(50, 45, Math.toRadians(0)))
                .waitDrive()

                //push the first sample in the hp station
                .driveTo(new Pose(10, 45, Math.toRadians(0)))
                .waitDrive()

                //return for the second sample
                .driveTo(new Pose(50, 45, Math.toRadians(0)))
                .waitDrive()
                .driveTo(new Pose(51, 58, Math.toRadians(0)))
                .waitDrive()

                //push the second sample in the hp station
                .driveTo(new Pose(10, 58, Math.toRadians(0)))
                .waitDrive()

                //let room for the hp to get the second sample
                .driveTo(new Pose(30, 46, Math.toRadians(0)))
                .waitDrive()
                .driveTo(new Pose(2, 46, Math.toRadians(0)))
                .waitDrive()

                //end auto
                .end();
    }

    @Override
    protected void InitializeThreads() {
    }

    @Override
    protected void Loop() {
    }
}
