package org.firstinspires.ftc.teamcode.Pathing.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants;
import org.firstinspires.ftc.teamcode.hardware.Generals.Enums;
import org.firstinspires.ftc.teamcode.hardware.Robot.ExoData;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Pose;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "RR_Localization",group = "RR")
public class
LocalizationTest extends LinearOpMode {
    CleverSwerve swerve;
    private Telemetry dashboardTelemetry;
    private Threaded_IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new CleverSwerve(this, CleverSwerve.Localizers.ROADRUNNER_TWO_WHEELS, Enums.OpMode.TELE_OP);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = new Threaded_IMU(this);

        waitForStart();

        swerve.setPoseEstimate(new Pose(0, 0, 0));
        ExoData constrains = new ExoData()
                .setFieldCentric(true);

        while (!isStopRequested()) {
            swerve.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, SwerveConstants.fastConstrain);
            swerve.read();
            swerve.update();

            Pose currentPose = swerve.getPoseEstimate();
            dashboardTelemetry.addLine("             POSE"                                       );
            dashboardTelemetry.addData("x:        ", currentPose.x                      );
            dashboardTelemetry.addData("y:        ", currentPose.y                      );
            dashboardTelemetry.addData("heading:  ", Math.toDegrees(currentPose.heading));
            dashboardTelemetry.addData("Angle: ", imu.getAngle(AngleUnit.DEGREES));

            dashboardTelemetry.update();
        }
    }
}