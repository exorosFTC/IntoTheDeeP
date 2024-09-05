package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IMU_Name;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

/**Test class for field-centric drive before mounting odometry wheels
 * Used strictly for the HEADING value**/
public class Threaded_IMU implements Localizer {
    private BNO055IMU imu;

    private Thread imuThread;
    private final Object imuAngleLock = new Object();
    private final Object imuVelocityLock = new Object();

    private double imuAngle = 0.0;
    private double imuVelocity = 0.0;

    private double imuOffset = 0.0;


    public Threaded_IMU getInstance() { return this; }

    public Threaded_IMU(LinearOpMode opMode) {
        synchronized (imuAngleLock) {
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_Name);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);
        }

        reset();
        startIMUThread(opMode);

    }

    private void startIMUThread(LinearOpMode opMode) {

            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested()) {
                    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    synchronized (imuAngleLock) { imuAngle = angles.secondAngle; }
                    synchronized (imuVelocityLock) { imuVelocity = imu.getAngularVelocity().zRotationRate; }
                }
            });

            imuThread.start();
    }


    /**The only useful value from here**/
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuAngle - imuOffset - Math.PI; }
            case DEGREES: { return fromRadiansToDegrees(imuAngle - imuOffset - Math.PI); }
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

    public void reset() { imuOffset = imuAngle; }

    public void setPositionEstimate(Pose newPoseEstimate) { imuOffset = newPoseEstimate.heading; }
}
