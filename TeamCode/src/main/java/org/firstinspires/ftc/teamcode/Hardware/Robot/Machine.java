package org.firstinspires.ftc.teamcode.Hardware.Robot;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastConstrainScalar;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowConstrainScalar;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.MecanumDrive;

import javax.annotation.Nullable;

public class Machine {
    public Hardware hardware;

    public MecanumDrive drive;

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
    private Telemetry telemetry;

    //default data
    private MachineData data = new MachineData()
            .add(Enums.OpMode.TELE_OP)
            .add(Enums.Telemetry.DASHBOARD)
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
        this.hardware = Hardware.getInstance(opMode.hardwareMap);
        this.opMode = opMode;

        drive = new MecanumDrive(opMode);

        telemetry = (data.telemetryType == Enums.Telemetry.REGULAR) ? opMode.telemetry :
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = add_g1 ? new GamepadEx(opMode.gamepad1) : null;
        g2 = add_g2 ? new GamepadEx(opMode.gamepad2) : null;

        openCvCamera = usingOpenCvCamera ? new Camera(opMode, telemetry) : null;
        aprilTagCamera = usingAprilTagCamera ? new AprilTagCamera(opMode, telemetry) : null;

        isGamepadTriggerPressed = !usingButtonSensitivity && usingVelocityToggle ?
                new Trigger(() -> g1.getTrigger(data.sensitivityTrigger) > 0.01) : null;


        if (data.sensitivityTrigger == null && data.sensitivityButton == null)
            usingDriveSensitivity = false;

        return this;
    }




    public void read() { hardware.read(); }

    public void write() {
        drive.write();
    }




    public void update() {
        updateGamepad();
        updateDriveSensitivity();
    }

    public void updateDriveSensitivity() {
        if (usingDriveSensitivity) {
            if (usingVelocityToggle) {
                if (usingButtonSensitivity && g1.wasJustPressed(data.sensitivityButton))
                    isFast = !isFast;
                else if(!usingButtonSensitivity) {

                    if (isGamepadTriggerPressed.get() && !lastTriggerValue)
                        isFast = !isFast;
                    lastTriggerValue = isGamepadTriggerPressed.get();

                }
                speed = isFast ? fastConstrainScalar : slowConstrainScalar;

            }
            else {
                speed = fastConstrainScalar;

                if (usingButtonSensitivity && g1.isDown(data.sensitivityButton))
                    speed = slowConstrainScalar;
                else if (!usingButtonSensitivity && g1.getTrigger(data.sensitivityTrigger) > 0.01)
                    speed = slowConstrainScalar;
            }
        }
    }

    public void updateGamepad() {
        if (g1 != null) g1.readButtons();
        if (g2 != null) g2.readButtons();
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
        if (telemetry != null) {
            telemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
            telemetry.update();
        }
    }

    public void addTelemetry(String caption, Object value) { if (telemetry != null) telemetry.addData(caption, value); }

    public void clearTelemetry() { if (telemetry != null) telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetry != null) {
            if (telemetryAddLoopTime) {
                getEndingLoopTime();
                telemetry.addData("Loop Time: ", getLoopFrequency());
                getStartingLoopTime();
            }
            telemetry.update();
        }
    }

    public void debuggingTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Drive", 0);
        }
    }
}
