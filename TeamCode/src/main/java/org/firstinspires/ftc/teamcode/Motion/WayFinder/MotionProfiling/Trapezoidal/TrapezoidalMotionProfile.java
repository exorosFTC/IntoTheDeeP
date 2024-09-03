package org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling.Trapezoidal;

import static org.firstinspires.ftc.teamcode.Motion.WayFinder.Math.MathFormulas.QuadraticSolve;
import static org.firstinspires.ftc.teamcode.Motion.WayFinder.Math.MathFormulas.RectangleArea;
import static org.firstinspires.ftc.teamcode.Motion.WayFinder.Math.MathFormulas.TriangleArea;

import org.firstinspires.ftc.teamcode.Hardware.Generals.MotionProfile;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling.ProfileConstrains;

public class TrapezoidalMotionProfile implements MotionProfile {
    private enum ProfileShape {
        TRAPEZOIDAL,
        TRIANGULAR
    }

    private double startPosition, endPosition;
    private double distance;

    private boolean reversed = false;
    private boolean isBusy = true;

    private ProfileConstrains constrains;
    private double MAX_vel, MAX_accel, MAX_decel;

    private double t1, t2, t3;
    private double dist1, dist2, dist3;
    private double total_time;

    private double max_reached_velocity;

    private ProfileShape shape;

    public TrapezoidalMotionProfile(double startPosition, double endPosition, ProfileConstrains constrains) {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.distance = Math.abs(endPosition - startPosition);

        if (endPosition < startPosition) {
            reversed = true;
        } else {
            reversed = false;
        }

        this.constrains = constrains;
        MAX_vel = constrains.MAX_VELOCITY;
        MAX_accel = constrains.MAX_ACCELERATION;
        MAX_decel = constrains.MAX_DECELERATION;

        calculateProfileShape();
    }

    private void calculateProfileShape() {
        t1 = MAX_vel / MAX_accel;
        t3 = MAX_vel / MAX_decel;

        if (t1 + t2 <= 2 * distance / MAX_vel)
            shape = ProfileShape.TRAPEZOIDAL;
        else shape = ProfileShape.TRIANGULAR;

        switch (shape) {
            case TRAPEZOIDAL: {
                t2 = (distance - (TriangleArea(MAX_vel, t1) + TriangleArea(MAX_vel, t3))) / MAX_vel;

                max_reached_velocity = MAX_vel;
            }
            break;

            case TRIANGULAR: {
                double a = (MAX_accel / 2) * (1 - MAX_accel / -MAX_decel);
                double b = 0;
                double c = -distance;

                double[] solutions;

                if (QuadraticSolve(a, b, c) != null) {
                    solutions = QuadraticSolve(a, b, c);
                } else solutions = new double[]{0};

                t1 = solutions[0];

                max_reached_velocity = MAX_accel * t1;

                t3 = max_reached_velocity / MAX_decel;
                t2 = 0;

            }
        }

        total_time = t1 + t2 + t3;
        dist1 = TriangleArea(max_reached_velocity, t1);
        dist2 = RectangleArea(max_reached_velocity, t2);
        dist3 = TriangleArea(max_reached_velocity, t3);
    }

    @Override
    public MotionState calculate(final double t) {
        if (isBusy) {
            double position = 0, velocity = 0, acceleration = 0, stageTime = 0;
            MotionState.Stage stage = null;

            isBusy = true;

            if (t <= t1) {
                stageTime = t;
                acceleration = MAX_accel;
                velocity = acceleration * t;
                position = TriangleArea(velocity, stageTime);

                stage = MotionState.Stage.T1;
            } else if (t <= t1 + t2) {
                stageTime = t - t1;
                acceleration = 0;
                velocity = max_reached_velocity;
                position = dist1 + RectangleArea(velocity, stageTime);

                stage = MotionState.Stage.T2;
            } else if (t <= total_time) {
                stageTime = total_time - (t1 + t2);
                acceleration = MAX_decel;
                velocity = acceleration * t;
                position = dist1 + dist2 + TriangleArea(velocity, stageTime);

                stage = MotionState.Stage.T3;
            } else {
                isBusy = false;
            }

            if (reversed)
                return new MotionState(position, -velocity, -acceleration, stage, stageTime);

            return new MotionState(position, velocity, acceleration, stage, stageTime);
        }

        return new MotionState();
    }

    @Override
    public boolean isBusy() { return isBusy; }

    public boolean isReversed() { return reversed; }

    @Override
    public ProfileConstrains getConstrains() { return constrains; }
}
