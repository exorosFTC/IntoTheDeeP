package org.firstinspires.ftc.teamcode.Motion.WayFinder.Math;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization.Pose;

import java.util.Arrays;
import java.util.List;

public class Transformations {

    public static Pose2d Pose_2_Pose2d(Pose pose) { return new Pose2d(pose.x, pose.y, pose.heading); }

    public static Pose Pose2d_2_Pose(Pose2d pose) { if (pose != null) return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    return new Pose(); }

    public static List<Double> doubleMatrix_2_doubleList(double[][] matrix) {
        return Arrays.asList(matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3]);
    }

    public static double inches_2_cm(double in) { return in * 2.54; }

    public static double cm_2_inches(double cm) { return cm / 2.54; }
}
