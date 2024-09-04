package org.firstinspires.ftc.teamcode.Pathing.WayFinder.Pathing.PathFollowers;

import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Pose;

import lombok.Data;

@Data public class MotionSignal {
    public Pose velocity;
    public Pose acceleration;
}
