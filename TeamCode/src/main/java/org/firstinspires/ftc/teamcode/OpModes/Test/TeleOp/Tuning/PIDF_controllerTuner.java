package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
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

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift.TwoMotorLift;


/**Manual TUNER class for linear sliders
 * Increase P until it gets to the target
 * Increase D to dampen the oscillations
 * Increase F to keep it in position (gravity feedforward)
 * DON'T MODIFY I !!!!!!
 */


@Config
@TeleOp(name = "PID", group = "tuning")
public class PIDF_controllerTuner extends LinearOpMode {
    public static double p = 0.01, d = 0;
    public static double f = 0;
    public static int target = 0;

    private TwoMotorLift extension;

    //TODO: rename the hardware calls in conformation with YOUR configuration
    private final String LEFT = LeftOuttakeMotor;
    private final String RIGHT = RightOuttakeMotor;

    private double startLoopTime, endLoopTime;
    private final double secondsToNanoseconds = 1000000000;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extension = new TwoMotorLift(this, LEFT, RIGHT);

        //TODO: configure your setup
        extension.addLimit(outtakeMAX)
                .addCurrentAlert(1)
                .addGearRatios(10.0 / 8, 8.0 / 8)
                .reverseRight();

        waitForStart();

        while (opModeIsActive()) {
            startLoopTime = endLoopTime;


            extension.setControllerPID(p, 0, d, f);
            extension.setPosition(target);

            endLoopTime = System.nanoTime();

            telemetry.addData("target ", target);
            telemetry.addData("Loop time: ", secondsToNanoseconds / (endLoopTime - startLoopTime));

            telemetry.update();
        }
    }

}
