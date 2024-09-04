package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math.Point;

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
