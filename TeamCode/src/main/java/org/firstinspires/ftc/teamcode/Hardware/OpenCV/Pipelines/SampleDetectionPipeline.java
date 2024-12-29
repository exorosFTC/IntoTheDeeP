package org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {

    private final Scalar lowerBlue = new Scalar(90, 100, 100);
    private final Scalar upperBlue = new Scalar(130, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 100, 20);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);

    private final Scalar lowerRed2 = new Scalar(160, 100, 20);
    private final Scalar upperRed2 = new Scalar(179, 255, 255);

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    Point center = new Point();

    @Override
    public Mat processFrame(Mat input) {
        Mat inputToHSV = new Mat();
        Mat detections = new Mat();



        /** convert from RGB to HSV */
        Imgproc.cvtColor(input, inputToHSV, Imgproc.COLOR_RGB2HSV);



        /** detect desired color */
        switch (SystemConstants.detectionColor) {
            case BLUE: { Core.inRange(inputToHSV, lowerBlue, upperBlue, detections); } break;
            case YELLOW: { Core.inRange(inputToHSV, lowerYellow, upperYellow, detections); } break;
            case RED: {
                Core.inRange(inputToHSV, lowerRed1, upperRed1, detections);
                Core.inRange(inputToHSV, lowerRed2, upperRed2, detections);
            } break;
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();



        /** find the contours of all detections */
        Imgproc.findContours(detections, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);



        /** find the largest contour*/
        Point cameraCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);

        MatOfPoint closestContour = null;
        Point closestCenter = null;
        double minDistanceSquared = Double.MAX_VALUE; // Use squared distance to avoid computing the square root

        for (MatOfPoint contour : contours) {
            Moments moments = Imgproc.moments(contour);

            // Skip contours with zero moments (avoid division by zero)
            if (moments.m00 != 0) {
                double centerX = moments.m10 / moments.m00;
                double centerY = moments.m01 / moments.m00;

                // Compute squared distance directly
                double distanceSquared = (centerX - cameraCenter.x) * (centerX - cameraCenter.x)
                        + (centerY - cameraCenter.y) * (centerY - cameraCenter.y);

                if (distanceSquared < minDistanceSquared) {
                    minDistanceSquared = distanceSquared;
                    closestContour = contour;
                    closestCenter = new Point(centerX, centerY);
                }
            }
        }



        /** if contour exits, draw the contour and it's center*/
        if (closestContour != null) {
            Imgproc.drawContours(input, Collections.singletonList(closestContour), -1, new Scalar(0, 255, 255), 3); // Yellow contour
            Imgproc.circle(input, closestCenter, 10, new Scalar(0, 0, 255), -1); // Red dot for the center
        } else { center = null; }



        /** also draw the camera center*/
        Imgproc.circle(input,  cameraCenter, 10, new Scalar(0, 0, 255), -1);



        /** clear cache */
        inputToHSV.release();
        detections.release();
        hierarchy.release();

        if (closestContour != null) closestContour.release();
        for (MatOfPoint mat : contours) { mat.release(); }



        /** display the processed image */
        return input;
    }

    public Point getCenter() { return center; }
}
