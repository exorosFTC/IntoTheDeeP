package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOuttakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOuttakeMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


/**Manual TUNER class for linear sliders
 * Increase P until it gets to the target
 * Increase D to dampen the oscillations
 * Increase F to keep it in position (gravity feedforward)
 * DON'T MODIFY I !!!!!!
 */


@Config
@TeleOp(name = "PID", group = "tuning")
public class PIDF_controllerTuner extends LinearOpMode {
    /**To be tuned: */
    private PIDController controller;
    private final double ticks_in_degree = 537.7 / 360;


    public static double p = 0.01, d = 0;
    public static double f = 0;
    public static int target = 0;

    private DcMotorEx leftMotor, rightMotor;

    //TODO: rename the hardware calls in conformation with YOUR configuration
    private String LEFT = LeftOuttakeMotor;
    private String RIGHT = RightOuttakeMotor;

    private double startLoopTime, endLoopTime;
    private double secondsToNanoseconds = 1000000000;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, LEFT);
        rightMotor = hardwareMap.get(DcMotorEx.class, RIGHT);

        //TODO: Reverse motors if necessary
        initializeMotor(leftMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initializeMotor(rightMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p, 0, d);

        waitForStart();

        while (opModeIsActive()) {
            startLoopTime = endLoopTime;

            int motorPositionL = leftMotor.getCurrentPosition();
            //int motorPositionR = rightMotor.getCurrentPosition();

            controller.setPID(p, 0, d);
            double pid = controller.calculate(motorPositionL, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;

            //controller.setPID(p, 0, d);
            //double pidR = controller.calculate(motorPositionR, target);
            //double powerR = pidR + ff;

            leftMotor.setPower(power);
            rightMotor.setPower(power);

            endLoopTime = System.nanoTime();

            telemetry.addData("Position Left:  ", motorPositionL);
            telemetry.addData("Position Right:  ", motorPositionL);
            telemetry.addData("POWER: ", power);
            telemetry.addData("target ", target);
            telemetry.addData("Loop time: ", secondsToNanoseconds / (endLoopTime - startLoopTime));

            telemetry.update();
        }
    }

    private void initializeMotor(DcMotorEx motor, DcMotor.RunMode runMode) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



}
