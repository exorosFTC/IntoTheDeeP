package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.accelerationScalar;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingAcceleration;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingExponentialInput;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class Drivetrain {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    private static double lastX = 0, lastY = 0, lastHead = 0;

    private double LF, RF, LB, RB;

    public Drivetrain(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

        hardware.motors.get(LeftBack).setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.motors.get(LeftFront).setDirection(DcMotorSimple.Direction.REVERSE);

        /**hardware.motors.get(LeftFront).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.motors.get(LeftBack).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.motors.get(RightFront).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.motors.get(RightBack).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        this.opMode = opMode;
    }

    /**input velocity NEEDS to be clamped to [-1, 1]*/
    public void update(Pose velocity) {
        if (usingExponentialInput)
            velocity = exponential(velocity);

        if (usingAcceleration)
            velocity = accelerate(velocity);

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



    private Pose exponential(Pose velocity) {
        return new Pose(Math.pow(velocity.x, 3),
                Math.pow(velocity.y, 3),
                Math.pow(velocity.heading, 3));
    }

    private Pose accelerate(Pose velocity) {
        if (velocity.x == 0) lastX = 0;
        else if (Math.abs(velocity.x - lastX) > accelerationScalar) {
            lastX += Math.signum(velocity.x) * accelerationScalar;
            velocity.x = lastX;
        }

        if (velocity.y == 0) lastY = 0;
        else if (Math.abs(velocity.y - lastY) > accelerationScalar) {
            lastY += Math.signum(velocity.y) * accelerationScalar;
            velocity.y = lastY;
        }

        if (velocity.heading == 0) lastHead = 0;
        else if (Math.abs(velocity.heading - lastHead) > accelerationScalar) {
            lastHead += Math.signum(velocity.heading) * accelerationScalar;
            velocity.heading = lastHead;
        }

        return velocity;
    }
}
