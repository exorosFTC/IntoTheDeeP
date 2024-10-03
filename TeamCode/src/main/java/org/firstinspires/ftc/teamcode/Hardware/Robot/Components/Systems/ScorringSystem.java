package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;

public class ScorringSystem {
    private static Hardware hardware;

    private Intake intake;
    private Outtake outtake;

    public ScorringSystem(LinearOpMode opMode) {
        intake = new Intake(opMode);
        outtake = new Outtake(opMode);
    }
}
