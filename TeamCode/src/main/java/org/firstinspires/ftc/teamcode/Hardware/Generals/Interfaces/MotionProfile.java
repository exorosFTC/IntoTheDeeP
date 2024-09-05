package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

import org.firstinspires.ftc.teamcode.Pathing.MotionProfiling.MotionState;
import org.firstinspires.ftc.teamcode.Pathing.MotionProfiling.ProfileConstrains;

public interface MotionProfile {
    boolean isBusy();

    ProfileConstrains getConstrains();

    MotionState calculate(final double t);

}
