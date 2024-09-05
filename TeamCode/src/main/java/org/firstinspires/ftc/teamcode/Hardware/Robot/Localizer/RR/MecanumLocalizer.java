package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR;



import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOdometry;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.Pose2d_2_Pose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.Pose_2_Pose2d;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;


import java.util.Arrays;
import java.util.List;

/**Roadrunner implementation of 3 odometry wheel localizer
 * @noinspection ALL*/
public class MecanumLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {
    private Hardware hardware;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.57; //to be tuned (inch)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //to be tuned (inch) - because roadrunner, duhh
    public static double LEFT_DISTANCE = -6.3;
    public static double RIGHT_DISTANCE = 4.67;
    public static double FRONT_DISTANCE = 2.75;

    public static double encoderTicksToInch(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public MecanumLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LEFT_DISTANCE, 0), // left
                new Pose2d(0, RIGHT_DISTANCE, 0), // right
                new Pose2d(FRONT_DISTANCE, 0, Math.toRadians(90)) // front/back/perpendicular
        ));

        hardware = Hardware.getInstance(hardwareMap);


        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInch(hardware.encoderReadings.get(LeftOdometry)),
                encoderTicksToInch(hardware.encoderReadings.get(RightOdometry)),
                encoderTicksToInch(hardware.encoderReadings.get(PerpendicularOdometry))
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInch(0),
                encoderTicksToInch(0),
                encoderTicksToInch(0)
        );
    }

    public Pose getRobotPosition() { return Pose2d_2_Pose(super.getPoseEstimate()); }

    @Nullable
    public Pose getRobotVelocity() { return Pose2d_2_Pose(super.getPoseVelocity()); }



    public void setPositionEstimate(Pose newPose) { super.setPoseEstimate(Pose_2_Pose2d(newPose)); }

    @Override
    public double getAngle(AngleUnit unit) {
        return 0;
    }

    @Override
    public void update() { super.update(); }

    public void read() {}
}
