package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous()
public class UltrasonicAlignment extends ExoMode {
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
    }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {
    }
}
