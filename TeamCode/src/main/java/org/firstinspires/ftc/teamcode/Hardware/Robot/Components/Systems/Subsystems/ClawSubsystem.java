package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;


public class ClawSubsystem {
    private Hardware hardware;

    private double
            open = 0,
            closed = 0,
            closed_partial = 0;

    private boolean isOpen = false;
    private String servoName;



    public ClawSubsystem(LinearOpMode opMode, String servoName) {
        this.hardware = Hardware.getInstance(opMode);
        this.servoName = servoName;
    }



    public ClawSubsystem addOpen(double val) {
        this.open = val;
        return this;
    }

    public ClawSubsystem addClosed(double val) {
        this.closed = val;
        return this;
    }

    public ClawSubsystem addClosedPartial(double val) {
        this.closed_partial = val;
        return this;
    }




    public void setOpen(boolean open) {
        hardware.servos.get(servoName).setPosition((open) ? this.open : closed);

    }

    public void setOpenPartial(boolean open) {
        hardware.servos.get(servoName).setPosition((open) ? this.open : closed_partial);
        this.isOpen = open;
    }

    public void toggleClaw() {
        this.isOpen = !isOpen;
        hardware.servos.get(servoName).setPosition((isOpen) ? open : closed);
    }

    public void toggleClawPartial() {
        this.isOpen = !isOpen;
        hardware.servos.get(servoName).setPosition((isOpen) ? open : closed_partial);

    }




}
