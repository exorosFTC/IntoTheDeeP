package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR;

import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.BL_motor;
import static org.firstinspires.ftc.teamcode.hardware.Generals.HardwareNames.FR_motor;
import static org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Transformations.Pose2d_2_Pose;
import static org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Transformations.Pose_2_Pose2d;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Generals.Localizer;
import org.firstinspires.ftc.teamcode.hardware.Robot.Swerve.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.Pathing.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Pose;

import java.util.Arrays;
import java.util.List;

import kotlin.Suppress;


public class TwoWheelMecanumLocalizer extends TwoTrackingWheelLocalizer implements Localizer {
    private static double TICKS_PER_REV = 8192;
    private static double WHEEL_RADIUS = 0.689;
    private static double GEAR_RATIO = 1;

    private static double X_multiplier = 1.175;
    private static double Y_multiplier = 1.164;

    private static double PARALLEL_X = 5.31; // X is the forward and back direction
    private static double PARALLEL_Y = 2.67; // Y is the strafe direction

    private static double PERPENDICULAR_X = 2.36; // X is the forward and back direction
    private static double PERPENDICULAR_Y = -2.87; // Y is the strafe direction

    private Encoder perpendicularEncoder, parallelEncoder;
    private Threaded_IMU imu;

    public TwoWheelMecanumLocalizer(LinearOpMode opMode){

        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        parallelEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, FR_motor)); //left encoder
        perpendicularEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, BL_motor));
        imu = new Threaded_IMU(opMode);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imu.getAngle(AngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_multiplier,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_multiplier
        );
    }

    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }

    @Override
    public double getAngle(AngleUnit unit) {
        return imu.getAngle(unit);
    }

    public void resetAngle() { imu.reset(); }

    @Override
    public void read() {}

    @Override
    public Pose getRobotPosition() { return Pose2d_2_Pose(super.getPoseEstimate()); }

    @Override
    public Pose getRobotVelocity() { return Pose2d_2_Pose(super.getPoseVelocity()); }

    @Override
    public void setPositionEstimate(Pose newPose) { super.setPoseEstimate(Pose_2_Pose2d(newPose)); }

    @Override
    public void update() { super.update(); }
}
