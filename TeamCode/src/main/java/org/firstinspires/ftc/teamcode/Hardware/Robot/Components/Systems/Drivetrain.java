package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_LENGTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class Drivetrain {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private double LF, RF, LB, RB;

    public Drivetrain(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode.hardwareMap);

        hardware.motors.get(LeftBack).setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.motors.get(LeftFront).setDirection(DcMotorSimple.Direction.REVERSE);

        this.opMode = opMode;
    }

    /**input velocity NEEDS to be clamped to [-1, 1]*/
    public void update(Pose velocity) {
        double x = velocity.x, y = velocity.y, head = velocity.heading;

        LF = x - y - head * TRACK_WIDTH;
        RF = x + y + head * TRACK_WIDTH;
        LB = x + y - head * TRACK_WIDTH;
        RB = x - y + head * TRACK_WIDTH;

        hardware.motors.get(LeftFront).setPower(LF);
        hardware.motors.get(LeftBack).setPower(LB);
        hardware.motors.get(RightFront).setPower(RF);
        hardware.motors.get(RightBack).setPower(RB);

    }

    public void update(double lf, double lb, double rf, double rb) {
        hardware.motors.get(LeftFront).setPower(lf);
        hardware.motors.get(LeftBack).setPower(lb);
        hardware.motors.get(RightFront).setPower(rf);
        hardware.motors.get(RightBack).setPower(rb);
    }
}
