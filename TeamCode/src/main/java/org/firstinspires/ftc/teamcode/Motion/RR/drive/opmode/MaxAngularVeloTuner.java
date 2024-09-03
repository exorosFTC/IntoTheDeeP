package org.firstinspires.ftc.teamcode.Motion.RR.drive.opmode;

import static org.firstinspires.ftc.teamcode.Motion.WayFinder.Math.Transformations.Pose_2_Pose2d;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(name = "RR_AngularVelocity",group = "RR")
public class MaxAngularVeloTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;
    CleverSwerve swerve;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.ROADRUNNER_THREE_WHEELS, Enums.OpMode.AUTONOMUS);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            swerve.drive(0, 0, 1, SwerveConstants.fastConstrain);

            Pose2d velocity = Objects.requireNonNull(Pose_2_Pose2d(swerve.getVelocityEstimate()), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(velocity.getHeading(), maxAngVelocity);
        }

        swerve.drive(0, 0, 0, SwerveConstants.fastConstrain);

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.addData("Max Recommended Angular Velocity (rad)", maxAngVelocity * 0.8);
        telemetry.addData("Max Recommended Angular Velocity (deg)", Math.toDegrees(maxAngVelocity * 0.8));
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}