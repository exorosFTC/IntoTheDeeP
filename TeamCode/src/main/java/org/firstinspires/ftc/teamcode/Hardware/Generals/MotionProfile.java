package org.firstinspires.ftc.teamcode.Hardware.Generals;

import org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.MotionProfiling.ProfileConstrains;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
