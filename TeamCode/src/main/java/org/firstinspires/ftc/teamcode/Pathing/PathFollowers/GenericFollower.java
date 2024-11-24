package org.firstinspires.ftc.teamcode.Pathing.PathFollowers;


import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.MotionProfile;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Paths;
import org.firstinspires.ftc.teamcode.Pathing.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;
import org.firstinspires.ftc.teamcode.Pathing.MotionProfiling.ProfileConstrains;
import org.firstinspires.ftc.teamcode.Pathing.MotionProfiling.Trapezoidal.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Pathing.PathBuilders.BezierCurve;
import org.firstinspires.ftc.teamcode.Pathing.PathBuilders.PurePursuit;

import java.util.List;

public class GenericFollower {
    private List<Paths> pathsToFollow;
    private Localizer localizer;
    private Paths currentBuildPath;
    private Paths currentFollowedPath;
    private MotionProfile profile;

    private MotionSignal signal = new MotionSignal();

    private boolean maintainHeading = false;
    private boolean isStarted;
    private boolean isPaused;
    private boolean isBusy;

    Pose currentRobotPose;

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower(Localizer localizer) {
        this.localizer = localizer;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower addPath() {
        currentBuildPath.build();
        pathsToFollow.add(currentBuildPath);

        return this;
    }

    public GenericFollower addPoint(Point newPoint) {
        if (currentBuildPath != null)
            currentBuildPath.addPoint(newPoint);

        return this;
    }

    public GenericFollower build() {
        if (!pathsToFollow.isEmpty())
            currentFollowedPath = pathsToFollow.get(0);
        else currentFollowedPath = null;

        currentBuildPath = null;
        isStarted = isPaused = isBusy = false;

        return this;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public GenericFollower newPurePursuit() {
        if (!currentBuildPath.wasBuilt()) { //if isn't built => you forgot to add last type of Path lmao
            currentBuildPath.build();
            pathsToFollow.add(currentBuildPath);
            currentBuildPath = null;
        }

        if (currentBuildPath == null)
            currentBuildPath = new PurePursuit(new Point());


        return this;
    }

    private GenericFollower IN_WORK_newBezierCurve() {
        if (!currentBuildPath.wasBuilt()) { //if isn't built => you forgot to add last type of Path lmao
            currentBuildPath.build();
            pathsToFollow.add(currentBuildPath);
            currentBuildPath = null;
        }

        if (currentBuildPath == null)
            currentBuildPath = new BezierCurve(new Pose());

        return this;
    }

    private GenericFollower IN_WORK_newTrapezoidalProfile(double start, double end, ProfileConstrains constrains) {
        profile = new TrapezoidalMotionProfile(start, end, constrains);

        return this;
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    public void start() {
        if (currentFollowedPath != null)
            currentFollowedPath.start();
    }

    public void pause() {
        if (currentFollowedPath != null)
            currentFollowedPath.pause();
    }

    public void resume() {
        if (currentFollowedPath != null)
            currentFollowedPath.resume();
    }

    public void cancel() {
        if (currentFollowedPath != null)
            currentFollowedPath.cancel();
    }

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    private void generateSignal() throws NotAPolynomialException {
        currentFollowedPath.update();

        Point currentFollowedPoint = currentFollowedPath.getPointToFollow();
        double heading = 0;

        if (!maintainHeading) { // you have 2 points and you want to face the target point, so you find the angle between the 2 points, easy
            heading = Math.atan2(currentFollowedPoint.y - currentRobotPose.y,
                    currentRobotPose.x - currentRobotPose.y);
        } else { heading = currentRobotPose.heading; } // just maintain heading, cuz why not

        signal.velocity = new Pose(currentFollowedPoint.subtract(currentRobotPose.getPoint()).multiplyBy(1), heading * 1);
    }

    //call this every loop
    public void update(){
        if (!currentFollowedPath.isBusy() && isStarted && !isPaused) {
            currentFollowedPath = pathsToFollow.iterator().hasNext() ? pathsToFollow.iterator().next() : null;
            if (currentFollowedPath != null)
                currentFollowedPath.start();
        }

        localizer.update();
        currentRobotPose = localizer.getRobotPosition();

        if (currentFollowedPath instanceof PurePursuit)
            ((PurePursuit) currentFollowedPath).setPose(currentRobotPose);

        try { generateSignal(); } catch (NotAPolynomialException e) {}
    }

    public boolean isBusy() {
        return currentFollowedPath != null && !isPaused;
    }

    public MotionSignal getMotionSignal() { return signal; }
    
    public void maintainHeading(boolean maintainHeading) { this.maintainHeading = maintainHeading; }
}
