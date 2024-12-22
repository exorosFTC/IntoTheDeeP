package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;

public class Follower {
    private Drivetrain drive;
    private ScorringSystem system;
    private Thread driveThread;
    private Hardware hardware;

    private LinearOpMode opMode;
    private ElapsedTime waitTimer;

    public Follower(LinearOpMode opMode, Machine robot) {
        this.drive = robot.drive;
        this.system = robot.system;

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    public Follower(LinearOpMode opMode, Drivetrain drive, ScorringSystem system) {
        this.drive = drive;
        this.system = system;

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    private void startDriveThread() {
        driveThread = new Thread(() -> {
            while (opMode.opModeIsActive()) {

                Pose currentPose = drive.localizer.getRobotPosition();
                drive.driveVector = new Pose(drive.linearC.calculate(currentPose.x, drive.target.x),
                        drive.linearC.calculate(currentPose.y, drive.target.y),
                        0);
                Point rotatedDiff = new Point(drive.driveVector.x, drive.driveVector.y).rotate_matrix(-Math.toRadians(currentPose.heading));

                drive.driveVector = new Pose(rotatedDiff.x,
                        -rotatedDiff.y,
                        drive.angularC.calculate(-FindShortestPath(Math.toRadians(currentPose.heading), drive.target.heading)));


                drive.update(drive.driveVector.multiplyBy(12 / hardware.batteryVoltageSensor.getVoltage()));
                drive.localizer.update();
                system.outtake.extension.update();

                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        });

        driveThread.start();
    }

    public Follower driveTo(Pose pose) {
        this.drive.hold(pose);
        return this;
    }

    public Follower waitDrive() {
        while (drive.isBusy() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public Follower waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public Follower moveSystem(Runnable action) {
        if (action != null) action.run();
        return this;
    }

    public Follower end() {
        driveThread.interrupt();
        drive.disable();
        return this;
    }




}
