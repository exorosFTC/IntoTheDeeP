package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

import org.firstinspires.ftc.teamcode.Pathing.Exceptions.NotAPolynomialException;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;

public interface Paths {
    void update() throws NotAPolynomialException;

    Paths addPoint(Point newPoint);
    void build();

    void start();
    void pause();
    void resume();
    void cancel();

    boolean isBusy();
    boolean wasBuilt();

    Point getPointToFollow();

}
