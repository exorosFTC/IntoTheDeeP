package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.Custom;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_WHEEL_RADIUS_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOdometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

/** @noinspection ALL*/
public class CustomMecanumLocalizer implements Localizer {
    private Hardware hardware;

    public double l = -14.25; //also cm
    public double r = 7.25;
    public double b = -24;

    private Pose currentPose, previousPose, velocityPose;

    private double previousL, previousR, previousB;
    private double currentL, currentR, currentB;
    private double delta_L, delta_R, delta_B;

    private final ElapsedTime loopTime;
    private final Dead3WheelLocalizer localizer;

    public static double encoderTicksToCM(double ticks) {
        return ODOMETRY_WHEEL_RADIUS_CM * 2 * Math.PI * ODOMETRY_GEAR_RATIO * ticks / ODOMETRY_TICKS_PER_REVOLUTION;
    }

    public CustomMecanumLocalizer(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode.hardwareMap);

        currentPose = new Pose();
        previousPose = new Pose();
        velocityPose = new Pose();
        loopTime = new ElapsedTime();
        previousL = previousR = previousB = 0;
        currentL = currentR = currentB = 0;

        localizer = new Dead3WheelLocalizer(l, r, b);

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @Override
    public void update() {

        Pose lastPose = currentPose;
        currentPose = localizer.getDeltaPoseEstimate(delta_L, delta_R, delta_B, currentPose);

        Pose deltaPose = currentPose.subtract(lastPose);
        velocityPose = deltaPose.divideBy(loopTime.seconds());
    }

    public Pose getRobotPosition() { return currentPose; }

    /**UNIT: m/s*/
    public Pose getRobotVelocity() { return velocityPose; }

    public void setPositionEstimate(Pose newPose) {
        currentPose = newPose;
        localizer.setStartOrientation(newPose.heading);
    }

    @Override
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return currentPose.heading; }
            case DEGREES: { return fromRadiansToDegrees(currentPose.heading); }
            default: { return 0; }
        }
    }

    public double getLeftEncodeValue() { return currentL; }

    public double getRightEncoderValue() { return currentR; }

    public double getBackEncoderValue() { return  currentB; }

    /**call this every loop*/
    public void read() {
        loopTime.reset();
        previousL = currentL;
        previousR = currentR;
        previousB = currentB;

        currentL = hardware.encoderReadings.get(LeftOdometry);
        currentR = hardware.encoderReadings.get(RightOdometry);
        currentB = hardware.encoderReadings.get(PerpendicularOdometry);


        delta_L = encoderTicksToCM(currentL - previousL);
        delta_R = encoderTicksToCM(currentR - previousR);
        delta_B = encoderTicksToCM(currentB - previousB);

        previousPose = currentPose;
    }

}
