package org.firstinspires.ftc.teamcode.Hardware.Generals;

import org.firstinspires.ftc.teamcode.Motion.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Motion.WayFinder.Localization.Point;

public interface Paths {
    void update() throws NotAPolynomialException;

    Paths addPoint(Point newPoint);
    void build();

    void start();
    void pause();
    void resume();

    boolean isBusy();

    Point getPointToFollow();

}
