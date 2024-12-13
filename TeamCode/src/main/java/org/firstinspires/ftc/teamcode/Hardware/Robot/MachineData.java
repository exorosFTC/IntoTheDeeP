package org.firstinspires.ftc.teamcode.Hardware.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

public class MachineData implements Enums, Enums.Mecanum {

    public Localizers localizer;
    public OpMode opModeType;
    public MotionPackage motionPackage;

    public GamepadKeys.Button sensitivityButton = null;
    public GamepadKeys.Trigger sensitivityTrigger = null;



    public MachineData add(Localizers localizer) {
        this.localizer = localizer;
        return this;
    }

    public MachineData add(OpMode opModeType) {
        this.opModeType = opModeType;
        return this;
    }

    public MachineData add(MotionPackage motionPackage) {
        this.motionPackage = motionPackage;
        return this;
    }



    public MachineData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public MachineData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public MachineData setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public MachineData setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

    public MachineData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }

}
