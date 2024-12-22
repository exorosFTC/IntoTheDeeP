package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous(name = "hold", group = "test")
public class AutoTuner extends ExoMode {
    private Drivetrain drive;
    private Hardware hardware;

    public static double linP = LinearP, angP = AngularP;
    public static double linD = LinearD, angD = AngularD;
    public static double x, y, head;

    @Override
    protected void Init() {
        drive = new Drivetrain(this);
        hardware = Hardware.getInstance(this);
    }

    @Override
    protected void WhenStarted() {
    }

    @Override
    protected void InitializeThreads() {

    }

    @Override
    protected void Loop() {
        drive.hold(new Pose(x, y, Math.toRadians(head)));

        drive.linearC.setPID(linP, 0, linD);
        drive.angularC.setPID(angP, 0, angD);

        Pose currentPose = drive.localizer.getRobotPosition();
        drive.driveVector = new Pose(drive.linearC.calculate(currentPose.x, drive.target.x),
                drive.linearC.calculate(currentPose.y, drive.target.y),
                0);
        Point rotatedDiff = new Point(drive.driveVector.x, drive.driveVector.y).rotate_matrix(-Math.toRadians(currentPose.heading));

        drive.driveVector = new Pose(rotatedDiff.x,
                -rotatedDiff.y,
                drive.angularC.calculate(-FindShortestPath(Math.toRadians(currentPose.heading), drive.target.heading)));


        drive.update(drive.driveVector.multiplyBy(12 / hardware.batteryVoltageSensor.getVoltage()));
        drive.localizer.update();

        hardware.telemetry.update();
        hardware.bulk.clearCache(Enums.Hubs.ALL);


        /**robot.addTelemetry("x: ", x);
        robot.addTelemetry("y: ", y);
        robot.addTelemetry("head: ", head);

        robot.addTelemetry("current x", Math.round(robot.localizer.getRobotPosition().x * 100) / 100);
        robot.addTelemetry("current y", Math.round(robot.localizer.getRobotPosition().y * 100) / 100);

        robot.addTelemetry("current heading (deg)", Math.toDegrees(robot.localizer.getAngle(AngleUnit.RADIANS)));
*/

    }
}
