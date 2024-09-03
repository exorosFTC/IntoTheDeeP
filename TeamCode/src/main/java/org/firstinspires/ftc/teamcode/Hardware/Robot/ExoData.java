package org.firstinspires.ftc.teamcode.Hardware.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Enums;

public class ExoData implements Enums, Enums.Mecanum {

    public Localizers localizer;
    public OpMode opModeType;
    public MotionPackage motionPackage;

    public GamepadKeys.Button sensitivityButton = null;
    public GamepadKeys.Trigger sensitivityTrigger = null;

    public ExoData add(Localizers localizer) {
        this.localizer = localizer;
        return this;
    }

    public ExoData add(OpMode opModeType) {
        this.opModeType = opModeType;
        return this;
    }

    public ExoData add(MotionPackage motionPackage) {
        this.motionPackage = motionPackage;
        return this;
    }


    public ExoData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public ExoData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public ExoData setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public ExoData setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

    public ExoData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }

}
