package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.MotorNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "ConfigureMotor", group = "tuning")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor, motor2;
    private List<String> names = Arrays.asList("FL", "FR", "BL", "BR");
    private GamepadEx g1;
    private Telemetry dashboardTelemetry;
    private int index = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, MotorNamesList.get(index));
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                index += 1;

                if (index == MotorNamesList.size())
                    index = 0;

                motor = hardwareMap.get(DcMotorEx.class, MotorNamesList.get(index));

            }

            if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            motor.setPower(-g1.getLeftY());

            dashboardTelemetry.addData("Motor: ", MotorNamesList.get(index));
            try { dashboardTelemetry.addData("Position: ", motor.getCurrentPosition());
            } catch (Exception e) {}

            dashboardTelemetry.update();

            g1.readButtons();
        }
    }
}
