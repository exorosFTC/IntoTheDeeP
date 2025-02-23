package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.basketPose;
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
public class BlueRight extends ExoMode {
    private Machine robot;
    private AutoDrive auto;

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
        driveSpecimens();

        auto.end();
    }

    private void preload() {
        auto
        // init scoring system
                .moveSystem(() -> {
            robot.system.outtake.openClaw(false);
            robot.system.intake.setAction(Enums.IntakeEnums.IntakeAction.TRANSFER);
            robot.hardware.servos.get(IntakeWrist).setPosition(0.55);
        })

                // move the outtake slightly just to be able to lift the arm and move on with your life
                .moveSystem(() -> { robot.system.specimens(); })

                // 1st basket drive. Aligning with odometry because ultrasonics suck
                .driveTo(new Pose(27, 0, Math.toRadians(0)))
                .waitDrive()

                .driveTillScoreSpecimen();

    }

    private void driveSpecimens() {
        auto
                .driveTo(new Pose())

                // 1st basket drive. Aligning with odometry because ultrasonics suck
                .driveTo(new Pose(10, 0, Math.toRadians(0)))
                .waitMs(300)
                .driveTo(new Pose(5, 40, Math.toRadians(0)))
                .waitDrive();
    }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {}
}
