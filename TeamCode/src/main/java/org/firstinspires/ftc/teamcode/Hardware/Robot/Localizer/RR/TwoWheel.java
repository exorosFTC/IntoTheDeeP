package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_LEFT_X_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_PERPENDICULAR_Y_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_WHEEL_RADIUS_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_X_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_Y_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toCustomPose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toIN;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toRoadrunnerPose;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU.Threaded_IMU;
import org.firstinspires.ftc.teamcode.Pathing.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.Arrays;
import java.util.List;


public class TwoWheel extends TwoTrackingWheelLocalizer implements Localizer {
    private Hardware hardware;
    private Encoder perpendicularEncoder, parallelEncoder;
    private Threaded_IMU imu;

    private static double WHEEL_RADIUS = toIN(ODOMETRY_WHEEL_RADIUS_CM);

    private static double PARALLEL_Y = toIN(ODOMETRY_LEFT_X_OFFSET_CM); // Y is the strafe direction
    private static double PERPENDICULAR_X = toIN(ODOMETRY_PERPENDICULAR_Y_OFFSET_CM); // X is the forward and back direction

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / ODOMETRY_TICKS_PER_REVOLUTION;
    }



    public TwoWheel(LinearOpMode opMode){

        super(Arrays.asList(
                new Pose2d(0, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, 0, Math.toRadians(90))
        ));

        hardware = Hardware.getInstance(opMode);

        parallelEncoder = new Encoder(hardware.motors.get(LeftOdometry));
        perpendicularEncoder = new Encoder(hardware.motors.get(PerpendicularOdometry));
        imu = new Threaded_IMU(opMode);
    }



    @Override
    public double getHeading() {
        return imu.getAngle(AngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * ODOMETRY_X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * ODOMETRY_Y_MULTIPLIER
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
    public Pose getRobotPosition() { return toCustomPose(super.getPoseEstimate()); }

    @Override
    public Pose getRobotVelocity() { return toCustomPose(super.getPoseVelocity()); }

    @Override
    public void setPositionEstimate(Pose newPose) { super.setPoseEstimate(toRoadrunnerPose(newPose)); }

    @Override
    public void update() { super.update(); }
}
