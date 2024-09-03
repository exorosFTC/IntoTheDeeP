package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureMotor", group = "tuning")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor, motor2;
    private List<String> names = Arrays.asList("motor", "FL", "FR", "BL", "BR");
    private GamepadEx g1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, names.get(0));
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motor2 = hardwareMap.get(DcMotorEx.class, names.get(5));
        //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(g1.getLeftY());
            //motor2.setPower(g1.getLeftY());
        }
    }
}
