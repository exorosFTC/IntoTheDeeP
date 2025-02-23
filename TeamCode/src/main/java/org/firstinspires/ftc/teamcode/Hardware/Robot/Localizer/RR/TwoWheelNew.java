package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.ODOMETRY_WHEEL_RADIUS_CM;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.forward;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.perpendicular;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftOdometry;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.PerpendicularOdometry;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toCustomPose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toIN;
import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toRoadrunnerPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU.SketchyIMU;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class TwoWheelNew implements Localizer {
    private Hardware hardware;
    private LinearOpMode opMode;

    private static double inPerTick = toIN(ODOMETRY_WHEEL_RADIUS_CM) * 2 * Math.PI / ODOMETRY_TICKS_PER_REVOLUTION;
    private static double tickPerIn = ODOMETRY_TICKS_PER_REVOLUTION / toIN(ODOMETRY_WHEEL_RADIUS_CM) * 2 * Math.PI;
    private Pose2d pose = new Pose2d(0, 0, 0);



    public static class Params {
        public double parYTicks = toIN(forward.y) * tickPerIn; // y position of the parallel encoder (in tick units)
        public double perpXTicks = toIN(perpendicular.x) * tickPerIn; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    public final SketchyIMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;


    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;






    public TwoWheelNew(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        par = new OverflowEncoder(new RawEncoder(hardware.motors.get(LeftOdometry)));
        perp = new OverflowEncoder(new RawEncoder(hardware.motors.get(PerpendicularOdometry)));

        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = new SketchyIMU(opMode);
    }




    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();


        AngularVelocity angularVelocityDegrees = imu.it.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        Rotation2d heading = Rotation2d.exp(imu.getAngle(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }

    @Override
    public Pose getRobotPosition() { return toCustomPose(pose); }

    @Override
    public void setPositionEstimate(Pose newPose) { pose = toRoadrunnerPose(newPose); }

    @Override
    public double getAngle(AngleUnit unit) { return imu.getAngle(unit); }
}
