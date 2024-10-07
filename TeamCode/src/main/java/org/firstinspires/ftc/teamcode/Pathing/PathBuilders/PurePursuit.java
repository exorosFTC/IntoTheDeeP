package org.firstinspires.ftc.teamcode.Pathing.PathBuilders;

import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.QuadraticSolve;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.findLinearFunction;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.toPower;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Paths;
import org.firstinspires.ftc.teamcode.Pathing.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Pathing.Math.Coefficients;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.ArrayList;
import java.util.List;

import javax.annotation.Nullable;

public class PurePursuit implements Paths {
    private List<Point> pathPoints = new ArrayList<Point>();

    private Point pointToFollow;

    private int waypoint = 0;

    private double radiusToSearch = 20; //cm
    //TODO: tune this

    private Pose pose;

    private boolean built = true;
    private boolean isStarted = false, isPaused = false, isBusy= false, isDone = false;
    private boolean useLastPointCorrection = false; //shifting the last point some small value from actual pose and use PID to get to final position

    private Point lastPointPID;
    private double lastThreePointsDifference = 0;
    private final double correction = 8; //cm (applies to x and y)
    //TODO: tune this too

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public PurePursuit(Point startPoint) { addPoint(startPoint); }

    public PurePursuit(Point startPoint, double radius) {
        this(startPoint);
        this.radiusToSearch = radius;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Override
    public Paths addPoint(Point newPoint) {
        pathPoints.add(newPoint);
        return this;
    }

    @Override
    public void build() {
        if (useLastPointCorrection) {
            lastThreePointsDifference = 1;
            addCorrectionLastPoint();
        }
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Override
    public void start() { isStarted = true; isBusy = true; }

    @Override
    public void pause() { isPaused = true; }

    @Override
    public void resume() { isPaused = false; }

    @Override
    public void cancel() { isStarted = false; isBusy = false; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /**Hierarchy for following points:
     *      -points found on next line
     *      -points found on the current line
     *      -point closer to the end of the line (in case of multiple solutions for the same line)
     *      (+ never going back - checking with last point followed)
     *
     *
     * */

    @Override
    public void update() throws NotAPolynomialException {
        if (!isStarted || !isBusy || isPaused) {
            pointToFollow = pose.getPoint();
            return;
        }

        int remainingPoints = pathPoints.size() - waypoint;

        if (remainingPoints > 2)        handleMultiplePoints();
        else if (remainingPoints == 2)  handleTwoPoints();
        else                            completePath();

    }

    private void handleMultiplePoints() throws NotAPolynomialException {

        // Check for intersections with the next line segment
        List<Point> intersectionsNextLine = findCircleIntersections(
                pathPoints.get(waypoint + 1),
                pathPoints.get(waypoint + 2));

        // Move to the next line segment if intersections are found
        if (intersectionsNextLine != null) {
            findClosestPointToEndOfTheLine(intersectionsNextLine, pathPoints.get(waypoint + 2));
            waypoint++;
        } else {
            // Check for intersections with the current line segment
            List<Point> intersectionsCurrentLine = findCircleIntersections(
                    pathPoints.get(waypoint),
                    pathPoints.get(waypoint + 1));

            // Update the following point if intersections are found
            if (intersectionsCurrentLine != null)
                findClosestPointToEndOfTheLine(intersectionsCurrentLine, pathPoints.get(waypoint + 1));

        }
    }

    private void handleTwoPoints() throws NotAPolynomialException {

        // Check for intersections with the current line segment
        List<Point> intersectionsCurrentLine = findCircleIntersections(
                pathPoints.get(waypoint),
                pathPoints.get(waypoint + 1));

        // Update the following point or advance the waypoint if no intersections are found
        if (intersectionsCurrentLine != null)
            findClosestPointToEndOfTheLine(intersectionsCurrentLine, pathPoints.get(waypoint + 1));
        else waypoint++;

    }

    private void completePath() {
        isBusy = false;
        isDone = true;
    }


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    @Nullable
    private List<Point> findCircleIntersections(Point point1, Point point2) throws NotAPolynomialException {
        Coefficients lineEcuationCoefficients = findLinearFunction(point1, point2);

        double lineEquation_a = lineEcuationCoefficients.getCoefficient(0).doubleValue();
        double lineEquation_b = lineEcuationCoefficients.getCoefficient(1).doubleValue();

        double a = toPower(lineEquation_a, 2) + 1;
        double b = 2 * (lineEquation_a * (lineEquation_b - pose.y) - pose.x);
        double c = (lineEquation_b - pose.y + radiusToSearch) * (lineEquation_b - pose.y - radiusToSearch) + toPower(pose.x, 2);

        double[] solutions = QuadraticSolve(a, b, c);
        if (solutions == null) { return null; }

        List<Point> intersectingPoints = new ArrayList<>();
        for(int i = 0; i < solutions.length; i++) {
            double pointX = solutions[i];
            double pointY = lineEquation_a * pointX + lineEquation_b;

            intersectingPoints.add(new Point(pointX, pointY));
        }

        return intersectingPoints;
    }

    private void findClosestPointToEndOfTheLine(List<Point> solutions, Point end) {
        if (solutions.size() == 2) {
            double firstPointDistance = distanceBetweenTwoPoints(solutions.get(0), end);
            double secondPointDistance = distanceBetweenTwoPoints(solutions.get(1), end);

            pointToFollow = (firstPointDistance < secondPointDistance) ? solutions.get(0) : solutions.get(1);
        } else pointToFollow = solutions.get(0);

    }

    private double distanceBetweenTwoPoints(Point point1, Point point2) {
        return Math.hypot(point2.x - point1.x, point2.y - point1.y);
    }

    private void addCorrectionLastPoint() {
        lastPointPID = pathPoints.get(pathPoints.size() -1);

        pathPoints.remove(pathPoints.get(pathPoints.size() - 1));
        pathPoints.add(new Point(lastPointPID.x - correction, lastPointPID.y - correction));
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void usingCorrection(boolean using) { this.useLastPointCorrection = using; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public Point getPointToFollow() { return pointToFollow; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public boolean isStarted() { return isStarted; }

    public boolean isPaused() { return isPaused; }

    @Override
    public boolean isBusy() { return isBusy; }

    public boolean isDone() { return isDone; }

    @Override
    public boolean wasBuilt() { return built; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void setPose(Pose position) { pose = position; }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
}

