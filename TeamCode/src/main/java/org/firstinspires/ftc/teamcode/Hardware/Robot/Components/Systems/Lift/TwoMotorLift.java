package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.accelerationLiftExtended;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.accelerationLiftRetracted;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.accelerationScalar;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.fastDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.slowDrive;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.manualLiftCoefficient;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeTicksPerDegree;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware.Init;

import java.util.HashMap;
import java.util.Map;

import javax.crypto.MacSpi;


public class TwoMotorLift {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    public final Map<String, Integer> values = new HashMap<>();

    private int MIN = 0, MAX = 0;
    private int position = 0;
    private double amps = 1.5;
    private int diff;

    private double gearRatioLeft = 1.0;
    private double gearRatioRight = 1.0;

    private static double p = 0, d = 0;
    private double f = 0;
    private final PIDController controller;

    private final String
        LEFT, RIGHT;



    public TwoMotorLift(LinearOpMode opMode, String left, String right) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        controller = new PIDController(p, 0, d);

        LEFT = left;
        RIGHT = right;

        resetEncoders();
    }


    public TwoMotorLift add(String key, int value) {
        values.put(key, Integer.valueOf(value));
        return this;
    }

    public TwoMotorLift addLimit(int MAX) {
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public TwoMotorLift addLimit(int MIN, int MAX) {
        this.MIN = MIN;
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public TwoMotorLift addGearRatios(double leftRatio, double rightRatio) {
        this.gearRatioLeft = leftRatio;
        this.gearRatioRight = rightRatio;

        return this;
    }

    public TwoMotorLift addCurrentAlert(double amps) {
        this.amps = amps;

        hardware.motors.get(LEFT).setCurrentAlert(amps, CurrentUnit.AMPS);
        hardware.motors.get(RIGHT).setCurrentAlert(amps, CurrentUnit.AMPS);

        return this;
    }

    public TwoMotorLift reverseLeft() {
        hardware.motors.get(LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        return this;
    }

    public TwoMotorLift reverseRight() {
        hardware.motors.get(RIGHT).setDirection(DcMotorSimple.Direction.REVERSE);
        return this;
    }

    public TwoMotorLift setPID(double p, double i, double d) {
        this.p = p;
        this.d = d;
        controller.setPID(p, 0, d);
        //fuck i

        return this;
    }


    public void resetEncoders() {
        hardware.motors.put(LEFT, Init.initializeMotor(hardware.motors.get(LEFT), DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        hardware.motors.put(RIGHT, Init.initializeMotor(hardware.motors.get(RIGHT), DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }



    public void setPosition(String key) { position = values.get(key).intValue();}

    public void setPosition(int pos) { position = Math.min(MAX, Math.max(MIN, pos)); }


    public void extend(double power) {
        extend(power, false);
    }

    // input [-1, 1] ---- joystick input
    public void extend(double power, boolean noLimit) {
        int newPosition = this.position + (int) (this.diff * manualLiftCoefficient * power);

        if (!noLimit)
            position = Math.min(MAX, Math.max(MIN, newPosition));
    }

    public int getPosition() {
        return hardware.motors.get(LEFT).getCurrentPosition();
    }

    public void setControllerPID(double p, double i, double d, double f) {
        controller.setPID(p, i, d);
        this.f = f;
    }

    public void disable() {
        hardware.motors.get(LEFT).setMotorDisable();
        hardware.motors.get(RIGHT).setMotorDisable();
    }

    public void autoReset() {
        while (opMode.opModeIsActive() && !constrained())
            extend(-1);
        extend(0);

       resetEncoders();
    }

    public boolean constrained() {
        return hardware.motors.get(LEFT).isOverCurrent() || hardware.motors.get(RIGHT).isOverCurrent();
    }

    public void update() {
        if (!reached(20)) {
            int currentPosition = hardware.motors.get(LEFT).getCurrentPosition();

            if (currentPosition > Enums.OuttakeEnums.LiftAction.TRANSFER.ticks + 600) {
                accelerationScalar = accelerationLiftExtended;
                MecanumConstants.speed = slowDrive;
            }
            else {
                MecanumConstants.speed = fastDrive;
                accelerationScalar = accelerationLiftRetracted;
            }

            //hardware.telemetry.addData("OuttakePosition: ", currentPosition);
            //hardware.telemetry.addData("TargetPosition: ", position);


            double pid = controller.calculate(currentPosition, position);
            double ff = Math.cos(Math.toRadians(position / outtakeTicksPerDegree)) * f;
            double power = pid + ff;

            power = Math.max(Math.min(power, 1), -1);

            //hardware.telemetry.addData("power: ", power);

            double leftPower = (gearRatioRight > 1) ? power / gearRatioRight : power * gearRatioRight;
            double rightPower = (gearRatioLeft > 1) ? power / gearRatioLeft : power * gearRatioLeft;

            hardware.motors.get(LEFT).setPower(leftPower);
            hardware.motors.get(RIGHT).setPower(rightPower);
        }
        else {
            hardware.motors.get(LEFT).setPower(0);
            hardware.motors.get(RIGHT).setPower(0);
        }
    }

    public boolean reached() { return reached(5); }

    public boolean reached(double threshold) {
        return Math.abs(hardware.motors.get(LEFT).getCurrentPosition() - position) <= threshold;
    }



}
