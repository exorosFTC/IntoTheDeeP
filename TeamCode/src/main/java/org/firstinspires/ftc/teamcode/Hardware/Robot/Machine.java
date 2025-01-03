package org.firstinspires.ftc.teamcode.Hardware.Robot;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.driveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.speed;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingButtonSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingDriveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingVelocityToggle;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingOpenCvCamera;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.ScorringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheel;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import javax.annotation.Nullable;

public class Machine {
    public Hardware hardware;
    //public Follower follower;
    public Thread drivetrainThread;

    public Drivetrain drive;
    public ScorringSystem system;


    public GamepadEx g1, g2;
    private boolean add_g1, add_g2;

    private double startLoopTime, endLoopTime;
    private final int secondsToNanoseconds = 1000000000;

    private boolean isFast = true;
    private Trigger isGamepadTriggerPressed;
    private boolean lastTriggerValue = false;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;


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
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        //follower = new Follower(hardware);
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

        if (SystemConstants.opModeType != Enums.OpMode.AUTONOMUS)
            createG1Thread();

        return this;
    }





    private void createG1Thread() {
        if (g1 != null)
            drivetrainThread = new Thread(() -> {
                while (opMode.opModeIsActive()) {
                    drive.update(new Pose(
                            g1.getLeftY() * speed,      // forwards
                            -g1.getLeftX() * speed,                  // sideways
                            -g1.getRightX() * driveSensitivity     // turning
                            ));

                    if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        system.hangG1();
                        drivetrainThread.interrupt();
                    }

                    /** B to actually SCORE*/
                    if (g1.wasJustPressed(GamepadKeys.Button.A))
                        system.outtake.setAction(Enums.OuttakeEnums.OuttakeAction.SCORE);

                    g1.readButtons();
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
        if (g2 != null) g2.readButtons();
    }




    public void updateSystem() {
        updateGamepad();
        updateDriveSensitivity();
        system.update();

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
        hardware.telemetry.addLine("INIT COMPLETE! KILL EM' ALL 😈");
        hardware.telemetry.update();
    }

    public void addTelemetry(String caption, Object value) { hardware.telemetry.addData(caption, value); }

    public void clearTelemetry() { hardware.telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetryAddLoopTime) {
            getEndingLoopTime();
            hardware.telemetry.addData("Loop Time: ", getLoopFrequency());
            getStartingLoopTime();
            }
        hardware.telemetry.update();
    }

    public void debuggingTelemetry() {
        hardware.telemetry.addData("Drive", speed);
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
