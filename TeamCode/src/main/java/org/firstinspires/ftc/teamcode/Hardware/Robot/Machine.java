package org.firstinspires.ftc.teamcode.Hardware.Robot;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.driveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingButtonSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingDriveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingVelocityToggle;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingOpenCvCamera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.ThreeWheel;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheel;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;
import org.firstinspires.ftc.teamcode.Pathing.PathFollowers.GenericFollower;

import javax.annotation.Nullable;

public class Machine {
    public Hardware hardware;
    public HubBulkRead bulk;
    public Thread drivetrainThread;

    private VoltageSensor batteryVoltageSensor;

    public Drivetrain drive;
    public ScorringSystem system;


    public GamepadEx g1, g2;
    private boolean add_g1, add_g2;

    private double startLoopTime, endLoopTime;
    private final int secondsToNanoseconds = 1000000000;

    private double speed;
    private boolean isFast = true;
    private Trigger isGamepadTriggerPressed;
    private boolean lastTriggerValue = false;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;

    public Localizer localizer;
    public GenericFollower follower;

    //default data
    private MachineData data = new MachineData()
            .add(Enums.OpMode.TELE_OP)
            .setUsingAprilTag(false)
            .setUsingOpenCv(false);




    public Machine addData(MachineData data) {
        this.data = data;
        return this;
    }

    public Machine addGamepads(Enums.Gamepads gamepads) {
        switch (gamepads) {
            case G1: {
                add_g1 = true;
                add_g2 = false;
            } break;

            case G2: {
                add_g1 = false;
                add_g2 = true;
            } break;

            case BOTH: {
                add_g1 = true;
                add_g2 = true;
            } break;
        }

        return this;
    }

    public Machine construct(LinearOpMode opMode) {
        this.bulk = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        drive = new Drivetrain(opMode);
        system = new ScorringSystem(opMode);

        g1 = add_g1 ? new GamepadEx(opMode.gamepad1) : null;
        g2 = add_g2 ? new GamepadEx(opMode.gamepad2) : null;

        openCvCamera = usingOpenCvCamera ? new Camera(opMode, hardware.telemetry) : null;
        aprilTagCamera = usingAprilTagCamera ? new AprilTagCamera(opMode, hardware.telemetry) : null;

        isGamepadTriggerPressed = !usingButtonSensitivity && usingVelocityToggle ?
                new Trigger(() -> g1.getTrigger(data.sensitivityTrigger) > 0.01) : null;


        if (data.sensitivityTrigger == null && data.sensitivityButton == null)
            usingDriveSensitivity = false;

        if (data.opModeType == Enums.OpMode.AUTONOMUS) {
            switch (data.localizer) {
                case TWO_WHEELS: localizer = new TwoWheel(opMode);
                case THREE_WHEELS: localizer = new ThreeWheel(opMode);
            }
            follower = new GenericFollower(localizer);
        }

        createDrivetrainThread();

        return this;
    }





    private void createDrivetrainThread() {
        if (g1 != null)
            drivetrainThread = new Thread(() -> {
                while (opMode.opModeIsActive()) {
                    drive.update(new Pose(
                            -g1.getLeftY() * 1,     // forwards
                            g1.getLeftX() * 1,      // sideways
                            -g1.getRightX() * driveSensitivity      // turning
                    ));
                    g1.readButtons();
                }
            });
        else if (g2 != null)
            drivetrainThread = new Thread(() -> {
                while (opMode.opModeIsActive()) {
                    drive.update(new Pose(
                            -g2.getLeftY() * 1,     // forwards
                            g2.getLeftX() * 1,      // sideways
                            -g2.getRightX() * driveSensitivity      // turning
                    ));
                    g2.readButtons();
                }
            });
    }

    private void updateDriveSensitivity() {
        if (usingDriveSensitivity) {
            if (usingVelocityToggle) {
                if (usingButtonSensitivity && g1.wasJustPressed(data.sensitivityButton))
                    isFast = !isFast;
                else if(!usingButtonSensitivity) {

                    if (isGamepadTriggerPressed.get() && !lastTriggerValue)
                        isFast = !isFast;
                    lastTriggerValue = isGamepadTriggerPressed.get();

                }
                speed = isFast ? fastDrive : slowDrive;

            }
            else {
                speed = fastDrive;

                if (usingButtonSensitivity && g1.isDown(data.sensitivityButton))
                    speed = slowDrive;
                else if (!usingButtonSensitivity && g1.getTrigger(data.sensitivityTrigger) > 0.01)
                    speed = slowDrive;
            }
        }
    }

    private void updateGamepad() {
        if (g1 != null) g1.readButtons();
        if (g2 != null) g2.readButtons();
    }


    public void updateDrive() { updateDrive(false); }

    public void updateDrive(boolean exponential) {
        if (exponential)
            drive.update(new Pose(
                    exp(-g1.getLeftY()) * 1,            // forwards
                    exp(g1.getLeftX()) * 1,             // sideways
                    exp(-g1.getRightX()) * driveSensitivity     // turning
            ));
        else
            drive.update(new Pose(
                -g1.getLeftY() * 1,     // forwards
                g1.getLeftX() * 1,      // sideways
                -g1.getRightX() * driveSensitivity      // turning
            ));
    }

    public void updateDrive(double x, double y, double head) {
        drive.update(new Pose(x, y, head));
    }

    public void updateSystem() {
        if (data.opModeType == Enums.OpMode.TELE_OP) {
            updateGamepad();
            updateDriveSensitivity();
        }
        system.update();

    }

    public double exp(double x) {
        return x * x * x;
    }




    private void getStartingLoopTime() { startLoopTime = endLoopTime; }

    private void getEndingLoopTime() { endLoopTime = System.nanoTime(); }

    public double getLoopFrequency() { return telemetryAddLoopTime ? secondsToNanoseconds / (endLoopTime - startLoopTime) : 0; }




    public double getClosestAprilTag() { return (aprilTagCamera == null) ? 0 : aprilTagCamera.getDetection(); }

    public void searchForProp() {
        if (openCvCamera == null || !(openCvCamera.getPipeline() instanceof PropDetectionPipeline)) { randomization = null; }
            else {
                Enums.Randomization detection = openCvCamera.getRandomization();

                if (detection != null)
                    randomization = detection;
        }
    }

    @Nullable
    public Enums.Randomization getPropPosition() { return (usingOpenCvCamera) ? randomization : null; }

    public void setPipeline(Enums.Pipelines pipeline) { if (usingOpenCvCamera) openCvCamera.setPipeline(pipeline); }

    public void closeCamera() { if (usingOpenCvCamera) openCvCamera.close(); }




    public void initComplete() {
        if (hardware.telemetry != null) {
            hardware.telemetry.addLine("INIT COMPLETE! KILL EM' ALL 😈");
            hardware.telemetry.update();
        }
    }

    public void addTelemetry(String caption, Object value) { if (hardware.telemetry != null) hardware.telemetry.addData(caption, value); }

    public void clearTelemetry() { if (hardware.telemetry != null) hardware.telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetryAddLoopTime) {
            getEndingLoopTime();
            hardware.telemetry.addData("Loop Time: ", getLoopFrequency());
            getStartingLoopTime();
            }
        hardware.telemetry.update();
    }

    public void debuggingTelemetry() {
        if (hardware.telemetry != null) {
            hardware.telemetry.addData("Drive", 0);
        }
    }



    public void setAccess(Enums.Access access) {
        switch (access) {
            case INTAKE: {
                opMode.gamepad2.rumble(0, 1, 500);
                opMode.gamepad2.setLedColor(0.96, 0.54,  0.09, 120000);
            } break;

            case OUTTAKE: {
                opMode.gamepad2.rumble(1, 0, 500);
                opMode.gamepad2.setLedColor(0.5, 0,  0.5, 120000);
            } break;
        }
    }
}
