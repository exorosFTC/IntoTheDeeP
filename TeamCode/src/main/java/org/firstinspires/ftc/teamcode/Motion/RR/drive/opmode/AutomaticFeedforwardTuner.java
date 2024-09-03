package org.firstinspires.ftc.teamcode.Motion.RR.drive.opmode;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SwerveConstants.fastConstrain;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Enums.Swerve.Localizers.CUSTOM;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve.CleverSwerve.rpmToVelocity;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve.SwerveModule.SwerveModule.MAX_RPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.Motion.RR.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.Motion.RR.util.RegressionUtil;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization.Pose;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(name = "RR_Feedforward", group = "RR")
public class AutomaticFeedforwardTuner extends LinearOpMode implements Enums.Swerve {
    public static double MAX_POWER = 0.7;
    public static double DISTANCE = 100; // in

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CleverSwerve swerve = new CleverSwerve(this, CUSTOM, Enums.OpMode.AUTONOMUS);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Would you like to fit kStatic?");
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetry.addLine("Press (Y/Δ) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && gamepad1.y) {
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        double maxVel = rpmToVelocity(MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        swerve.setPoseEstimate(new Pose());

        double startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(swerve.getPoseEstimate().x);
            powerSamples.add(power);

            swerve.drive(power, 0.0, 0.0, fastConstrain);
        }

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitIntercept,
                LoggingUtil.getLogFile(Misc.formatInvariant(
                        "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clearAll();
        telemetry.addLine("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetry.addLine("Would you like to fit kA?");
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitAccelFF = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.clearAll();
            telemetry.addLine("Place the robot back in its starting position");
            telemetry.addLine("Press (Y/Δ) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.y) {
                idle();
            }
            while (!isStopRequested() && gamepad1.y) {
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE / maxVel;

            timeSamples.clear();
            positionSamples.clear();
            powerSamples.clear();

            swerve.setPoseEstimate(new Pose());


            startTime = clock.seconds();
            while (!isStopRequested()) {
                swerve.drive(MAX_POWER, 0.0, 0.0, fastConstrain);
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(swerve.getPoseEstimate().x);
                powerSamples.add(MAX_POWER);

                swerve.update();
            }

            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                    timeSamples, positionSamples, powerSamples, rampResult,
                    LoggingUtil.getLogFile(Misc.formatInvariant(
                            "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clearAll();
            telemetry.addLine("Constant power test complete");
            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}