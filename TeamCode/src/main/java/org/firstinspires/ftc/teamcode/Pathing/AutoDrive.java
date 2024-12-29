package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheel;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;

public class AutoDrive {
    /**CHANGE THESE 2 WITH YOUR SPECIFIC HARDWARE CLASSES*/
    private final Drivetrain drive;
    private final ScorringSystem system;
    private final Camera camera;




    private Thread driveThread;
    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private final Hardware hardware;
    private final Localizer localizer;

    private final PIDController linearC, angularC;
    private double busyThreshold = 0.08;

    private Pose target = new Pose();
    private Pose driveVector = new Pose();

    private boolean isPaused = false;
    private boolean previousIsPaused = false;




    public AutoDrive(LinearOpMode opMode, Machine robot) {
        if (SystemConstants.usingOpenCvCamera) {
            this.camera = new Camera(opMode, robot.hardware.telemetry);
            camera.setPipeline(Enums.Pipelines.DETECTING_SAMPLE);
        } else camera = null;

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
                updateLiftPID();
                localizer.update();
                hardware.bulk.clearCache(Enums.Hubs.ALL);

                if (isPaused && !previousIsPaused) {    // stop the robot when paused
                    drive.update(new Pose());
                    previousIsPaused = true;
                    continue;
                }

                if (isPaused) continue;

                Pose currentPose = localizer.getRobotPosition();
                double radians = Math.toRadians(currentPose.heading);

                driveVector = new Pose(
                        linearC.calculate(currentPose.x, target.x),
                        linearC.calculate(currentPose.y, target.y),
                        0);
                Point rotatedDiff = driveVector.point().rotate_matrix(-radians);

                driveVector = new Pose(
                        rotatedDiff.x,
                        -rotatedDiff.y,
                        angularC.calculate(-FindShortestPath(radians, target.heading)));


                drive.update(driveVector.multiplyBy(12 / hardware.batteryVoltageSensor.getVoltage()));
            }
        });

        driveThread.start();
    }




    public AutoDrive pause() {
        isPaused = true;
        return this;
    }

    public AutoDrive resume() {
        isPaused = false;
        previousIsPaused = false;
        return this;
    }

    public AutoDrive driveTo(Pose pose) {
        this.target = pose;
        return this;
    }

    public AutoDrive waitDrive() {
        while (isBusy() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive moveSystem(Runnable action) {
        if (action != null) action.run();
        return this;
    }

    public AutoDrive setBusyThreshold(double threshold) {
        this.busyThreshold = threshold;
        return this;
    }

    public AutoDrive setPose(Pose pose) {
        localizer.setPositionEstimate(pose);
        return this;
    }

    public AutoDrive alignToSample() {
        if (camera == null) return this;

        Point error = camera.getSampleCenterError();

        Point rotatedError = error
                .rotate_matrix(-Math.toRadians(system.intake.getRotationAngle()))       // intake rotation offset
                .rotate_matrix(-Math.toRadians(localizer.getRobotPosition().heading));  // robot heading offset

        target = target.sum(rotatedError);

        return this;
    }

    public AutoDrive end() {
        driveThread.interrupt();
        drive.disable();
        return this;
    }





    public void setLinearPID(double p, double i, double d) { linearC.setPID(p, i, d); }

    public void setAngularPID(double p, double i, double d) { angularC.setPID(p, i, d); }

    public boolean isBusy() { return !driveVector.closeToZero(busyThreshold); }

    private void updateLiftPID() { system.outtake.extension.update(); }

}
