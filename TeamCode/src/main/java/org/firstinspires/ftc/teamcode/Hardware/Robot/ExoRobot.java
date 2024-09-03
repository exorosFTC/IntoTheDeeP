package org.firstinspires.ftc.teamcode.Hardware.Robot;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastConstrain;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowConstrain;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingButtonSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingDriveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingVelocityToggle;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingOpenCvCamera;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization.Pose;

import javax.annotation.Nullable;

public class ExoRobot {
    public GamepadEx g1, g2;
    private boolean add_g1, add_g2;

    private boolean override = false, overrideRotation = false;

    private VoltageSensor batteryVoltageSensor;
    public static double batteryVoltage;
    private HubBulkRead core;
    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;

    private double speed;
    private boolean isFast = true;
    private Trigger isGamepadTriggerPressed;
    private boolean lastTriggerValue = false;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;
    private Telemetry telemetry;

    //default constrains
    private ExoData constrains = new ExoData()
            .add(Enums.OpMode.TELE_OP)
            .setUsingAprilTag(false)
            .setUsingOpenCv(false);




    public ExoRobot addConstrains(ExoData constrains) {
        this.constrains = constrains;
        return this;
    }

    public ExoRobot addTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        return this;
    }

    public ExoRobot addGamepads(Enums.Gamepads gamepads) {
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

    public ExoRobot construct(LinearOpMode opMode) {
        this.opMode = opMode;

        g1 = add_g1 ? new GamepadEx(opMode.gamepad1) : null;
        g2 = add_g2 ? new GamepadEx(opMode.gamepad2) : null;

        openCvCamera = usingOpenCvCamera ? new Camera(opMode, telemetry) : null;
        aprilTagCamera = usingAprilTagCamera ? new AprilTagCamera(opMode, telemetry) : null;

        isGamepadTriggerPressed = !usingButtonSensitivity && usingVelocityToggle ?
                new Trigger(() -> g1.getTrigger(constrains.sensitivityTrigger) > 0.01) : null;

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        //core = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.AUTO);

        if (constrains.sensitivityTrigger == null && constrains.sensitivityButton == null)
            usingDriveSensitivity = false;


        setRobotInstance();

        return this;
    }




    public void read() {
        batteryVoltage = batteryVoltageSensor.getVoltage();

        if (g1 != null) g1.readButtons();
        if (g2 != null) g2.readButtons();

    }

    public void gamepadRumble(Gamepad gamepad, Enums.Rumbles type) {}

    public void updateAll() { }





    public void clearBulkCache() { core.clearCache(Enums.Hubs.ALL); }

    public Pose getRobotPosition() { return new Pose(); }

    public double getAngle() { return 0; }

    public double getBatteryVoltage() { return batteryVoltage; }

    public void setRobotPosition(Pose newPose) { }

    public void setRobotInstance() { }


    public void drive(double x, double y, double heading) {

        if (usingDriveSensitivity) {
            if (usingVelocityToggle) {
                if (usingButtonSensitivity && g1.wasJustPressed(constrains.sensitivityButton))
                    isFast = !isFast;
                else if(!usingButtonSensitivity) {

                    if (isGamepadTriggerPressed.get() && !lastTriggerValue)
                        isFast = !isFast;
                    lastTriggerValue = isGamepadTriggerPressed.get();

                }
                speed = isFast ? fastConstrain : slowConstrain;

            }
            else {
                speed = fastConstrain;

                if (usingButtonSensitivity && g1.isDown(constrains.sensitivityButton))
                    speed = slowConstrain;
                else if (!usingButtonSensitivity && g1.getTrigger(constrains.sensitivityTrigger) > 0.01)
                    speed = slowConstrain;
            }
        }


    }

    public void getStartingLoopTime() { startLoopTime = endLoopTime; }

    public void getEndingLoopTime() { endLoopTime = System.nanoTime(); }

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



    public void initCompleate() {
        if (telemetry != null) {
            telemetry.addLine("INIT COMPLETE! PRESS PLAY TO KILL EM' ALL");
            telemetry.update();
        }
    }

    public void addTelemetry(String caption, Object value) { if (telemetry != null) telemetry.addData(caption, value); }

    public void clearTelemetry() { if (telemetry != null) telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetry != null) {
            if (telemetryAddLoopTime)
                telemetry.addData("Loop Time: ", getLoopFrequency());
            telemetry.update();
        }
    }

    public void debuggingTelemetry() {
        if (telemetry != null) {

        }
    }
}
