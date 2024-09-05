package org.firstinspires.ftc.teamcode.Hardware.Robot.Components;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_LENGTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class MecanumDrive {
    private static Hardware hardware;

    private final double sum;
    private double LF, RF, LB, RB;

    public MecanumDrive(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap);

        this.sum = (TRACK_LENGTH + TRACK_WIDTH) / 2;
    }

    /**input velocity NEEDS to be clamped to [-1, 1]*/
    public void update(Pose velocity) {
        double x = velocity.x, y = velocity.y, head = velocity.heading;

        LF = x - y - head * sum;
        RF = x + y + head * sum;
        LB = x + y - head * sum;
        RB = x - y + head * sum;
    }

    public void write() {
        hardware.motors.get(LeftFront).setPower(LF);
        hardware.motors.get(LeftBack).setPower(LB);
        hardware.motors.get(RightFront).setPower(RF);
        hardware.motors.get(RightBack).setPower(RB);

        LF = LB = RF = RB = 0;
    }
}
