package org.firstinspires.ftc.teamcode.Pathing.Math;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

public class Transformations {

    public static Pose2d toRoadrunnerPose(Pose pose) {
        if (pose != null)
            return new Pose2d(
                    toIN(pose.x),
                    toIN(pose.y),
                    Math.toRadians(pose.heading));
        return new Pose2d();
    }

    // from inches and radians to cm and degrees
    public static Pose toCustomPose(Pose2d pose) {
        if (pose != null)
            return new Pose(toCM(pose.getX()),
                            toCM(pose.getY()),
                            Math.toDegrees(pose.getHeading()));
    return new Pose();
    }



    public static double toCM(double in) { return in * 2.54; }

    public static double toIN(double cm) { return cm / 2.54; }
}
