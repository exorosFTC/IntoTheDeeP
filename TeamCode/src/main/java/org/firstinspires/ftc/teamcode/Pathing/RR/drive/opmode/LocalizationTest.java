package org.firstinspires.ftc.teamcode.Pathing.RR.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "RR_Localization",group = "RR")
public class
LocalizationTest extends LinearOpMode {
    Machine robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.Telemetry.DASHBOARD)
                        .add(Enums.OpMode.TELE_OP)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);

        waitForStart();
        robot.localizer.setPositionEstimate(new Pose());


        while (!isStopRequested()) {

            robot.updateDrive();
            robot.localizer.update();

            Pose currentPose = robot.localizer.getRobotPosition();
            robot.addTelemetry("             POSE",""                                     );
            robot.addTelemetry("x:        ", currentPose.x                      );
            robot.addTelemetry("y:        ", currentPose.y                      );
            robot.addTelemetry("heading:  ", Math.toDegrees(currentPose.heading));

            robot.updateTelemetry();
        }
    }
}