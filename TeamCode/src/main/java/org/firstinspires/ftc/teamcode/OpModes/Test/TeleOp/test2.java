package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "test", name = "intakeTest")
public class test2 extends LinearOpMode {
    private GamepadEx g1;
    @Override
    public void runOpMode() throws InterruptedException {
        g1 = new GamepadEx(gamepad1);

        CRServo left = hardwareMap.get(CRServo.class,"Left"),
                right = hardwareMap.get(CRServo.class, "Right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            left.setPower(g1.getLeftY() - g1.getRightY());
            right.setPower(g1.getLeftY() - g1.getRightY());
            g1.readButtons();
        }
    }

    public double clip(double value) {
        return (value + 1) / 2;
    }
}
