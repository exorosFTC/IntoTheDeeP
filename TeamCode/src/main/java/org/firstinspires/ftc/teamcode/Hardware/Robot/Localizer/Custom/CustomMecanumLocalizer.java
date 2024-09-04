package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.Custom;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_WHEEL_RADIUS_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightOdometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Pose;

public class CustomMecanumLocalizer implements Localizer {
    public double l = -14.25; //also cm
    public double r = 7.25;
    public double b = -24;

    private Pose currentPose, previousPose, velocityPose;

    private double previousL, previousR, previousB;
    private double currentL, currentR, currentB;
    private double delta_L, delta_R, delta_B;

    private final ElapsedTime loopTime;

    private final Encoder leftEncoder, rightEncoder, backEncoder;
    private final Dead3WheelLocalizer localizer;

    public static double encoderTicksToCM(double ticks) {
        return ODOMETRY_WHEEL_RADIUS_CM * 2 * Math.PI * ODOMETRY_GEAR_RATIO * ticks / ODOMETRY_TICKS_PER_REVOLUTION;
    }

    public CustomMecanumLocalizer(HardwareMap hardwareMap) {
        currentPose = new Pose();
        previousPose = new Pose();
        velocityPose = new Pose();
        loopTime = new ElapsedTime();
        previousL = previousR = previousB = 0;
        currentL = currentR = currentB = 0;

        localizer = new Dead3WheelLocalizer(l, r, b);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LeftOdometry));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RightOdometry));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, PerpendicularOdometry));

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @Override
    public void update() {

        Pose lastPose = currentPose;
        currentPose = localizer.getDeltaPoseEstimate(delta_L, delta_R, delta_B, currentPose);
        //currentPose = new Pose(currentPose.sum(deltaPose.getPoint()), deltaPose.heading);

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

        currentL = leftEncoder.getCurrentPosition();
        currentR = rightEncoder.getCurrentPosition();
        currentB = backEncoder.getCurrentPosition();

        delta_L = encoderTicksToCM(currentL - previousL);
        delta_R = encoderTicksToCM(currentR - previousR);
        delta_B = encoderTicksToCM(currentB - previousB);

        previousPose = currentPose;
    }

}
