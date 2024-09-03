package org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling;

public class MotionState {
    public enum Stage {
        T1, T2, T3, T4, T5, T6, T7, UNDEFINED
    }

    public enum val {
        POSITION,
        VELOCITY,
        ACCELERATION,
        JERK,
        TIME
    }


    private double current_position = 0;
    private double current_velocity = 0;
    private double current_acceleration = 0;
    private double current_jerk = 0;

    private Stage current_stage = Stage.UNDEFINED;
    private double current_time_in_respective_stage = 0;

    public MotionState() {}

    public MotionState(double p, double v, double a, double j, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = a;
        this.current_jerk = j;

        this.current_stage = stage;
        this.current_time_in_respective_stage = t;
    }

    public MotionState(double p, double v, double a, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = a;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_respective_stage = t;
    }

    public MotionState(double p, double v, Stage stage, double t) {
        this.current_position = p;
        this.current_velocity = v;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_respective_stage = t;
    }

    public MotionState(double v, Stage stage, double t) {
        this.current_position = 0;
        this.current_velocity = v;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_respective_stage = t;
    }

    public MotionState(Stage stage, double t) {
        this.current_position = 0;
        this.current_velocity = 0;
        this.current_acceleration = 0;
        this.current_jerk = 0;

        this.current_stage = stage;
        this.current_time_in_respective_stage = t;
    }


    public double get(val value) {
        switch (value) {
            case POSITION: { return current_position; }
            case VELOCITY: { return current_velocity; }
            case ACCELERATION: { return current_acceleration; }
            case JERK: { return current_jerk; }
            case TIME: { return current_time_in_respective_stage; }
            default: { return 0; }
        }
    }

    public Stage getStage() { return current_stage; }

}
