package org.firstinspires.ftc.teamcode.Hardware.Util.SensorEx;

import static org.firstinspires.ftc.teamcode.hardware.Generals.Constants.SwerveConstants.VALUE_REJECTION;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

//ty kookybotz <33
public class CleverEncoder {
    private static double range = 3.3; //volts (V)
    private static double TAU = 2 * Math.PI;

    private double MAX = 720;
    private double MIN = 0;

    private double accumulatedValue = 0;
    private double lastPosition = 0, currentPosition;
    private double delta;

    private final AnalogInput encoder;
    private double offset = 0;
    private boolean reversed = false;

    private double voltage;
    private double position, last;

    public CleverEncoder(HardwareMap hardwareMap, String encoder_name) { this(hardwareMap, encoder_name, range); }

    public CleverEncoder(HardwareMap hardwareMap, String encoder_name, double actualRange) {
        encoder = hardwareMap.get(AnalogInput.class, encoder_name);
        this.range = actualRange;
    }

    public CleverEncoder zero() {
        setOffset(getPosition());
        return this;
    }

    public void reversed(boolean reversed) { this.reversed = reversed; }

    public boolean isReversed() { return reversed; }

    public double getPosition() {

        position = Angle.norm((!reversed ? 1 - getVoltage() / range : getVoltage() / range) * Math.PI - offset);

        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(Angle.normDelta(last)) > 0.1 || Math.abs(Angle.normDelta(position)) < 1) last = position;

        return last;
    }

    public double getAccumulativePosition() {
        currentPosition = getPosition();
        delta = currentPosition - lastPosition;

        if (Math.abs(delta) >= Math.PI) {
            if (lastPosition <= TAU && currentPosition >= 0) {
                delta = Math.PI - lastPosition + currentPosition;
            } else if (lastPosition >= 0 && currentPosition <= TAU) {
                delta = -1 * (Math.PI - currentPosition + lastPosition);
            }
        }

        lastPosition = currentPosition;

        accumulatedValue = normalize(accumulatedValue + delta);
        return accumulatedValue;
    }

    private double normalize(double value) {
        if (value >= TAU)
            value -= TAU;
        else if (value < 0)
            value += TAU;

        return value;
    }

    public void setOffset(double offset) { this.offset = offset; }

    public AnalogInput getEncoder() { return encoder; }

    public double getVoltage() { return voltage; }

    public void read() { voltage = encoder.getVoltage(); }

    //debugging

    public double getCurrentPosition() { return currentPosition; }

    public double getLastPosition() { return lastPosition; }

    public double getDelta() { return delta; }
}
