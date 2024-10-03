package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.manualLiftCoefficient;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;

import java.util.HashMap;
import java.util.Map;


public class TwoMotorLift {
    private static Hardware hardware;
    public final Map<String, Integer> values = new HashMap<>();

    private int MIN = 0, MAX = 0;
    private int diff;

    private final String
        LEFT, RIGHT;

    public TwoMotorLift(Hardware hardware, String left, String right) {
        this.hardware = hardware;

        LEFT = left;
        RIGHT = right;

        hardware.motors.get(LEFT).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(RIGHT).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.motors.get(LEFT).setPower(1);
        hardware.motors.get(RIGHT).setPower(1);

        hardware.motors.get(RIGHT).setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void setPosition(String key) {
        int position = Math.min(MAX, Math.max(MIN, values.get(key).intValue()));

        hardware.motors.get(LEFT).setTargetPosition(position);
        hardware.motors.get(RIGHT).setTargetPosition(position);
    }

    // input [-1, 1] ---- joystick input
    public void extend(double power) {
        int pastPosition = hardware.motors.get(LEFT).getCurrentPosition();
        int position = pastPosition + (int) (this.diff * manualLiftCoefficient * power);

        position = Math.min(MAX, Math.max(MIN, position));

        hardware.motors.get(LEFT).setTargetPosition(position);
        hardware.motors.get(RIGHT).setTargetPosition(position);
    }

    public int getPosition() { return hardware.motors.get(LEFT).getCurrentPosition(); }

    public void disable() {
        hardware.motors.get(LEFT).setMotorDisable();
        hardware.motors.get(RIGHT).setMotorDisable();
    }
}
