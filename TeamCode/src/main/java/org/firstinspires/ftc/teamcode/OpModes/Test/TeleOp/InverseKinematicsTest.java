package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.extensionMultiplier;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeDistance;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeRightPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;

@Config
@TeleOp (name = "InverseTest", group = "test")
public class InverseKinematicsTest extends LinearOpMode {
    private Outtake outtake;
    private Hardware hardware;

    public static double multiplier = 0.8;
    public double target_distance = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(this);
        outtake = new Outtake(this);

        outtake.setArmAction(Outtake.ArmAction.PRE_TRANSFER);
        double position = 0;

        waitForStart();

        while (opModeIsActive()) {
            double distance = hardware.distance.get(OuttakeDistance).getDistance(DistanceUnit.MM);
            double error = distance - target_distance;

            double pos = -gamepad1.left_stick_y;

            if (pos < 0) pos = 0;

            extensionMultiplier = multiplier;
            outtake.inverseKinematics(160 * pos);

            hardware.telemetry.addData("Distance: ", distance);
            hardware.telemetry.update();

        }
    }
}
