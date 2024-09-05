package org.firstinspires.ftc.teamcode.Pathing.PathFollowers;

import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import lombok.Data;

@Data public class MotionSignal {
    public Pose velocity;
    public Pose acceleration;
}
