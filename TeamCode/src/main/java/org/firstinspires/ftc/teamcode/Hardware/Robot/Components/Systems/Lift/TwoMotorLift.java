package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.manualLiftCoefficient;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeTicksPerDegree;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware.Init;

import java.util.HashMap;
import java.util.Map;


public class TwoMotorLift {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    public final Map<String, Integer> values = new HashMap<>();

    private int MIN = 0, MAX = 0;
    private double amps = 1.5;
    private int diff;

    private double gearRatioLeft = 1.0;
    private double gearRatioRight = 1.0;

    private final double p = 0, d = 0, f = 0;
    private PIDController controller;

    private final String
        LEFT, RIGHT;



    public TwoMotorLift(LinearOpMode opMode, String left, String right) {
        this.hardware = Hardware.getInstance(opMode.hardwareMap, opMode.telemetry);
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


    public void resetEncoders() {
        hardware.motors.put(LEFT, Init.initializeMotor(hardware.motors.get(LEFT), DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        hardware.motors.put(RIGHT, Init.initializeMotor(hardware.motors.get(RIGHT), DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }



    public void setPosition(String key) {
        int position = Math.min(MAX, Math.max(MIN, values.get(key).intValue()));
        int currentPosition = hardware.motors.get(LEFT).getCurrentPosition();

        double pid = controller.calculate(currentPosition, position);
        double ff = Math.cos(Math.toRadians(position / outtakeTicksPerDegree)) * f;
        double power = pid + ff;

        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;

        double leftPower = (gearRatioRight > 1) ? power / gearRatioRight : power * gearRatioRight;
        double rightPower = (gearRatioLeft > 1) ? power / gearRatioLeft : power * gearRatioLeft;

        hardware.motors.get(LEFT).setPower(leftPower);
        hardware.motors.get(RIGHT).setPower(rightPower);
    }

    public void setPosition(int pos) {
        int position = Math.min(MAX, Math.max(MIN, pos));
        int currentPosition = hardware.motors.get(LEFT).getCurrentPosition();

        double pid = controller.calculate(currentPosition, position);
        double ff = Math.cos(Math.toRadians(position / outtakeTicksPerDegree)) * f;
        double power = pid + ff;

        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;

        double leftPower = (gearRatioRight > 1) ? power / gearRatioRight : power * gearRatioRight;
        double rightPower = (gearRatioLeft > 1) ? power / gearRatioLeft : power * gearRatioLeft;

        hardware.motors.get(LEFT).setPower(leftPower);
        hardware.motors.get(RIGHT).setPower(rightPower);
    }



    // input [-1, 1] ---- joystick input
    public void extend(double power) {
        int pastPosition = hardware.motors.get(LEFT).getCurrentPosition();
        int position = pastPosition + (int) (this.diff * manualLiftCoefficient * power);

        setPosition(position);
    }

    public int getPosition() {
        return hardware.motors.get(LEFT).getCurrentPosition();
    }

    public void disable() {
        hardware.motors.get(LEFT).setMotorDisable();
        hardware.motors.get(RIGHT).setMotorDisable();
    }

    public void autoReset() {
        while (opMode.opModeIsActive() && !constrained())
            extend(-1);

       resetEncoders();
    }

    public boolean constrained() {
        return hardware.motors.get(LEFT).isOverCurrent() || hardware.motors.get(RIGHT).isOverCurrent();
    }


}
