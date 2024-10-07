package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR;



import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_LEFT_X_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_PERPENDICULAR_Y_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_RIGHT_X_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_WHEEL_RADIUS_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOdometry;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toCustomPose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toIN;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toRoadrunnerPose;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.Encoder;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;


import java.util.Arrays;
import java.util.List;

/**Roadrunner implementation of 3 odometry wheel localizer**/
public class ThreeWheel extends ThreeTrackingWheelLocalizer implements Localizer {
    private Hardware hardware;
    private Encoder left, right, middle;

    public static double WHEEL_RADIUS = toIN(ODOMETRY_WHEEL_RADIUS_CM); //to be tuned (inch)

    //to be tuned (inch) - because roadrunner, duhh
    public static double LEFT_DISTANCE = toIN(ODOMETRY_LEFT_X_OFFSET_CM);
    public static double RIGHT_DISTANCE = toIN(ODOMETRY_RIGHT_X_OFFSET_CM);
    public static double FRONT_DISTANCE = toIN(ODOMETRY_PERPENDICULAR_Y_OFFSET_CM);

    public static double encoderTicksToInch(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / ODOMETRY_TICKS_PER_REVOLUTION;
    }



    public ThreeWheel(LinearOpMode opMode) {
        super(Arrays.asList(
                new Pose2d(0, LEFT_DISTANCE, 0), // left
                new Pose2d(0, RIGHT_DISTANCE, 0), // right
                new Pose2d(FRONT_DISTANCE, 0, Math.toRadians(90)) // front/back/perpendicular
        ));

        hardware = Hardware.getInstance(opMode.hardwareMap);

        left = new Encoder(hardware.motors.get(LeftOdometry));
        right = new Encoder(hardware.motors.get(RightOdometry));
        middle = new Encoder(hardware.motors.get(PerpendicularOdometry));


        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }



    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInch(left.getCurrentPosition()),
                encoderTicksToInch(right.getCurrentPosition()),
                encoderTicksToInch(middle.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInch(left.getCorrectedVelocity()),
                encoderTicksToInch(right.getCorrectedVelocity()),
                encoderTicksToInch(middle.getCorrectedVelocity())
        );
    }



    // also transforms inches to cm, thank me later <3
    public Pose getRobotPosition() { return toCustomPose(super.getPoseEstimate()); }

    @Nullable
    public Pose getRobotVelocity() { return toCustomPose(super.getPoseVelocity()); }

    @Override
    public double getAngle(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES)
            return Math.toDegrees(super.getPoseEstimate().getHeading());
        else return super.getPoseEstimate().getHeading();
    }



    public void setPositionEstimate(Pose newPose) { super.setPoseEstimate(toRoadrunnerPose(newPose)); }

    @Override
    public void update() { super.update(); }
}
