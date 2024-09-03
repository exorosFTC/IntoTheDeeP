package org.firstinspires.ftc.teamcode.Motion.WayFinder.Pathing.PathFollowers;

import org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization.Pose;

import lombok.Data;

@Data public class MotionSignal {
    public Pose velocity;
    public Pose acceleration;
}
