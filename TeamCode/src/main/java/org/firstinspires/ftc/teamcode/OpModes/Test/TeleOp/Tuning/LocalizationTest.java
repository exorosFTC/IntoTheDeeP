package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;

@TeleOp(name = "RR Localizer", group = "tuning")
public class LocalizationTest extends LinearOpMode {
    Machine robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        waitForStart();

        robot.drivetrainThread.start();

        while (opModeIsActive()) {
            robot.drive.localizer.update();
            robot.hardware.bulk.clearCache(Enums.Hubs.ALL);

            telemetry.addData("x", Math.round(robot.drive.localizer.getRobotPosition().x * 100) / 100);
            telemetry.addData("y", Math.round(robot.drive.localizer.getRobotPosition().y * 100) / 100);



            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.localizer.getAngle(AngleUnit.RADIANS)));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

    }
}