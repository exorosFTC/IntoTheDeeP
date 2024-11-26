package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

@TeleOp (name = "InverseTest", group = "test")
public class InverseKinematicsTest extends LinearOpMode {
    private Outtake outtake;
    private Hardware hardware;

    private double target_distance = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(this.hardwareMap, this.telemetry);
        outtake = new Outtake(this);

        outtake.setArmAction(Enums.Outtake.ArmAction.PRE_TRANSFER);
        double position = 0;

        waitForStart();

        while (opModeIsActive()) {
            double distance = hardware.color.get(OuttakeColor).getDistance(DistanceUnit.MM);
            double error = distance - target_distance;

            double pos = -gamepad1.left_stick_y;

            if (pos < 0) pos = 0;

            outtake.inverseKinematics(60 * pos);

            /*double joint1 = hardware.servos.get(OuttakeLeftPivot).getPosition();
            double addition = joint1 * 24 / 40;
            double joint2 = 0.7 - addition;*/


            /*hardware.servos.get(OuttakeWrist).setPosition(joint2);
            /**if (distance < 60 && Math.abs(error) > 3) {
                outtake.inverseKinematics(position + error);
            }*/

            /*hardware.telemetry.addData("joint1", joint1);
            hardware.telemetry.addData("joint2", joint2);*/
            hardware.telemetry.update();

        }
    }
}
