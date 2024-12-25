package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheel;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;

public class AutoBase {
    /**CHANGE THESE 2 WITH YOUR SPECIFIC HARDWARE CLASSES*/
    private final Drivetrain drive;
    private final ScorringSystem system;




    private Thread driveThread;
    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private final Hardware hardware;
    private final Localizer localizer;

    private final PIDController linearC, angularC;
    private double busyThreshold = 0.12;

    private Pose target = new Pose();
    private Pose driveVector = new Pose();




    public AutoBase(LinearOpMode opMode, Machine robot) {
        this.drive = robot.drive;
        this.system = robot.system;

        linearC = new PIDController(LinearP, 0, LinearD);
        angularC = new PIDController(AngularP, 0, AngularD);

        localizer = new TwoWheel(opMode);
        localizer.setPositionEstimate(new Pose());

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    private void startDriveThread() {
        driveThread = new Thread(() -> {
            while (opMode.opModeIsActive()) {

                Pose currentPose = localizer.getRobotPosition();
                driveVector = new Pose(
                        linearC.calculate(currentPose.x, target.x),
                        linearC.calculate(currentPose.y, target.y),
                        0);
                Point rotatedDiff = new Point(driveVector.x, driveVector.y).rotate_matrix(-Math.toRadians(currentPose.heading));

                driveVector = new Pose(
                        rotatedDiff.x,
                        -rotatedDiff.y,
                        angularC.calculate(-FindShortestPath(Math.toRadians(currentPose.heading), target.heading)));


                drive.update(driveVector.multiplyBy(12 / hardware.batteryVoltageSensor.getVoltage()));
                localizer.update();
                updateLiftPID();

                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        });

        driveThread.start();
    }




    public AutoBase driveTo(Pose pose) {
        this.target = pose;
        return this;
    }

    public AutoBase waitDrive() {
        while (isBusy() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoBase waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoBase moveSystem(Runnable action) {
        if (action != null) action.run();
        return this;
    }

    public AutoBase setBusyThreshold(double threshold) {
        this.busyThreshold = threshold;
        return this;
    }

    public AutoBase setPose(Pose pose) {
        localizer.setPositionEstimate(pose);
        return this;
    }

    public AutoBase end() {
        driveThread.interrupt();
        drive.disable();
        return this;
    }



    public void setLinearPID(double p, double i, double d) { linearC.setPID(p, i, d); }

    public void setAngularPID(double p, double i, double d) { angularC.setPID(p, i, d); }

    public boolean isBusy() {
        return !driveVector.closeToZero(busyThreshold);
    }

    private void updateLiftPID() { system.outtake.extension.update(); }

}
