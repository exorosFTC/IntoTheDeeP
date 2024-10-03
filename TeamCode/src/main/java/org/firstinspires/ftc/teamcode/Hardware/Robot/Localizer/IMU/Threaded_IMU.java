package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU;

import static org.apache.commons.math3.util.MathUtils.normalizeAngle;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.LOGO_DIRECTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.USB_DIRECTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IMU_Name;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

/**Test class for field-centric drive before mounting odometry wheels
 * Used strictly for the HEADING value**/
public class Threaded_IMU implements Localizer {
    private IMU imu;

    private Thread imuThread;
    private final Object imuAngleLock = new Object();
    private final Object imuVelocityLock = new Object();

    private double imuAngle = 0.0;
    private double imuVelocity = 0.0;

    private double imuOffset = 0.0;


    public Threaded_IMU getInstance() { return this; }

    public Threaded_IMU(LinearOpMode opMode) {
        synchronized (imuAngleLock) {
        imu = opMode.hardwareMap.get(IMU.class, IMU_Name);
        imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION))
        );
        }

        reset();
        startIMUThread(opMode);

    }

    private void startIMUThread(LinearOpMode opMode) {

            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested()) {
                    Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    synchronized (imuAngleLock) { imuAngle = normalizeAngle(-angles.thirdAngle, FastMath.PI); }
                    synchronized (imuVelocityLock) { imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate; }
                }
            });

            imuThread.start();
    }


    /**The only useful value from here**/
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuAngle; }
            case DEGREES: { return fromRadiansToDegrees(imuAngle); }
            default: { return 0; }
        }
    }

    public double getVelocity(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuVelocity; }
            case DEGREES: { return fromRadiansToDegrees(imuVelocity); }
            default: { return 0; }
        }
    }

    public Pose getRobotPosition() { return new Pose(0, 0, getAngle(AngleUnit.RADIANS)); }

    public Pose getRobotVelocity() { return new Pose(0, 0, getVelocity(AngleUnit.RADIANS)); }


    public void read() {}

    public void update() {}

    public void reset() { imu.resetYaw(); }

    public void setPositionEstimate(Pose newPoseEstimate) { imuOffset = newPoseEstimate.heading; }
}
