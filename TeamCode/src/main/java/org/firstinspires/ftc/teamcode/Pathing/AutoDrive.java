package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.UltraLinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.UltraLinearP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.basketPose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheelNew;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public class AutoDrive {
    /**CHANGE THESE 2 WITH YOUR SPECIFIC HARDWARE CLASSES*/
    private final Drivetrain drive;
    private final ScorringSystem system;



    private Thread driveThread;
    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private final Hardware hardware;
    private final Localizer localizer;

    private final PIDController linearC, angularC;
    private final PIDController ultraLinearC;
    private double busyThreshold = 0.14;

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose position = new Pose();
    private Pose target = new Pose();
    private Pose driveVector = new Pose();


    private boolean isPaused = false;
    private boolean previousIsPaused = false;
    private boolean usingFailSafe = false;




























    public AutoDrive(LinearOpMode opMode, Machine robot, Pose startPose) {
        this.drive = robot.drive;
        this.system = robot.system;
        this.position = startPose;

        linearC = new PIDController(LinearP, 0, LinearD);
        angularC = new PIDController(AngularP, 0, AngularD);

        ultraLinearC = new PIDController(UltraLinearP, 0, UltraLinearD);

        localizer = new TwoWheelNew(opMode);
        setPose(position);

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    public AutoDrive(LinearOpMode opMode, Machine robot) {
        this.drive = robot.drive;
        this.system = robot.system;

        linearC = new PIDController(LinearP, 0, LinearD);
        angularC = new PIDController(AngularP, 0, AngularD);

        ultraLinearC = new PIDController(UltraLinearP, 0, UltraLinearD);

        localizer = new TwoWheelNew(opMode);
        setPose(position);

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }



    private void startDriveThread() {
        driveThread = new Thread(() -> {
            try {
                while (opMode.opModeIsActive()) {
                    localizer.update();
                    position = localizer.getRobotPosition();
                    system.outtake.extension.update();

                    hardware.telemetry.addData("x: ", position.x);
                    hardware.telemetry.addData("y: ", position.y);
                    hardware.telemetry.addData("head: ", position.heading);
                    hardware.telemetry.addData("left: ", drive.left.getFilteredDistance(DistanceUnit.INCH));
                    hardware.telemetry.addData("right: ", drive.right.getFilteredDistance(DistanceUnit.INCH));

                    hardware.telemetry.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);

                    if (isPaused && !previousIsPaused) {    // stop the robot when paused
                        drive.update(new Pose());
                        previousIsPaused = true;
                        continue;
                    }

                    if (isPaused) continue;

                    updateDriveVector();

                    if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                        driveTo(position);

                    drive.update(driveVector);
                }
            } catch (RuntimeException e) {};
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
        this.failSafeTimeMs = Double.POSITIVE_INFINITY;
        this.usingFailSafe = false;
        this.target = pose;
        return this;
    }

    public AutoDrive driveTo(Pose pose, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = true;
        this.target = pose;

        failSafeTimer.reset();

        return this;
    }

    public AutoDrive validateBasket(double leftDistance, double rightDistance) {
        double
                correctionX = drive.right.getFilteredDistance(DistanceUnit.INCH) - rightDistance,
                correctionY = drive.left.getFilteredDistance(DistanceUnit.INCH) - leftDistance;

        // correct location based on the ultrasonic input
        setPose(new Pose(basketPose.x + correctionX, basketPose.y + correctionY, Math.toRadians(position.heading)));

        return this;
    }

    public AutoDrive driveTillScoreSpecimen() {
        pause();

        waitTimer.reset();
        while (opMode.opModeIsActive() && drive.front.getFilteredDistance(DistanceUnit.MM) > 20 && waitTimer.time(TimeUnit.MILLISECONDS) < 1200) {
            drive.update(new Pose(0.4, 0, 0));
        }

        drive.update(new Pose());
        system.score();
        driveTo(new Pose(position.x, position.y, Math.toRadians(position.heading)));
        resume();

        return this;
    }




    public AutoDrive waitDrive() { return waitDrive(opMode::idle); }

    public AutoDrive waitDrive(Runnable inLoop) {
        updateDriveVector();
        while (isBusy() && opMode.opModeIsActive()) { inLoop.run(); }

        driveTo(new Pose(position.x, position.y, Math.toRadians(position.heading)));

        return this;
    }

    public AutoDrive waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitAction(Supplier<Boolean> action) {
        while (!action.get() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitAction(Supplier<Boolean> action, Runnable inLoop) {
        while (!action.get() && opMode.opModeIsActive()) { inLoop.run(); }
        return this;
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
                                            Runnable inLoop,
                                            double ms,
                                            Runnable failSafe) {
        boolean useFailSafe = true;
        waitTimer.reset();

        do {
            inLoop.run();
            if (action.get()) useFailSafe = false;
        } while(!action.get() && opMode.opModeIsActive() && waitTimer.time(TimeUnit.MILLISECONDS) < ms);

        if (useFailSafe)
            failSafe.run();

        return this;
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
                                            double ms,
                                            Runnable failSafe) {
         return waitActionTimeFailSafe(action,
                opMode::idle,
                ms,
                failSafe);
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
                                            double ms) {
        return waitActionTimeFailSafe(action,
                opMode::idle,
                ms,
                opMode::idle);
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
        localizer.setPositionEstimate(
                new Pose(pose.x, pose.y, Math.toDegrees(pose.heading))
        );
        return this;
    }





    public AutoDrive disableWheelMotors() {
        drive.disable();
        return this;
    }

    public void end() {
        driveThread.interrupt();
        drive.disable();

        system.outtake.extension.disable();
        opMode.stop();
    }




    private void updateDriveVector() {
        double radians = Math.toRadians(position.heading);

        double targetVelX = linearC.calculate(position.x, target.x);
        double targetVelY = linearC.calculate(position.y, target.y);
        double targetVelHeading = angularC.calculate(-FindShortestPath(radians, target.heading));

        double velocityMagnitude = Math.sqrt(targetVelX * targetVelX + targetVelY * targetVelY);
        double turnRadius = velocityMagnitude / (Math.abs(targetVelHeading) + 1e-6);
        double centripetalAcc = velocityMagnitude * velocityMagnitude / (turnRadius + 1e-6);

        double driftCompensationX = -centripetalAcc * Math.sin(radians);
        double driftCompensationY = centripetalAcc * Math.cos(radians);

        targetVelX += driftCompensationX;
        targetVelY += driftCompensationY;

        // convert the adjusted vector to the field-centric coordinate system.
        Pose vector = new Pose(targetVelX, targetVelY, 0);
        Point rotatedDiff = vector.point().rotate_matrix(-radians);

        // build the final drive vector using the rotated linear outputs and the angular command.
        driveVector = new Pose(
                rotatedDiff.x,
                -rotatedDiff.y,
                targetVelHeading).multiplyBy(13.0 / hardware.batteryVoltageSensor.getVoltage());
    }





    public void setLinearPID(double p, double i, double d) { linearC.setPID(p, i, d); }

    public void setUltraLinearPID(double p, double i, double d) { ultraLinearC.setPID(p, i, d); }

    public void setAngularPID(double p, double i, double d) { angularC.setPID(p, i, d); }



    public Pose getPosition() { return position; }

    public boolean isBusy() { return !driveVector.closeToZero(busyThreshold); }




}
