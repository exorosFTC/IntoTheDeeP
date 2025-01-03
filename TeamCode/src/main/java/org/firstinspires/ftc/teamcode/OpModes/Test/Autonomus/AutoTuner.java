package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous(name = "MovementPIDTuner", group = "tuning")
public class AutoTuner extends ExoMode {
    private Machine robot;
    private AutoDrive auto;

    public static double linP = LinearP, angP = AngularP;
    public static double linD = LinearD, angD = AngularD;
    public static double x, y, head;

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
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, robot);
    }

    @Override
    protected void InitializeThreads() {

    }

    @Override
    protected void Loop() {
        auto.driveTo(new Pose(x, y, Math.toRadians(head)));

        auto.setLinearPID(linP, 0, linD);
        auto.setAngularPID(angP, 0, angD);
    }
}
