package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@TeleOp(name = "hang", group = "test")
public class HangRawTest extends LinearOpMode {
    private DcMotorEx LEFT, RIGHT;
    private GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        LEFT = hardwareMap.get(DcMotorEx.class, LeftOuttakeMotor);
        RIGHT = hardwareMap.get(DcMotorEx.class, RightOuttakeMotor);

        g2 = new GamepadEx(gamepad2);

        LEFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RIGHT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RIGHT.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            double power = -g2.getLeftY();

            if (g2.wasJustPressed(GamepadKeys.Button.B))
            {
                LEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LEFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            LEFT.setPower(power);
            RIGHT.setPower(power * 0.8);

            telemetry.addData("position:", LEFT.getCurrentPosition());

            telemetry.update();
            g2.readButtons();
        }

    }
}
