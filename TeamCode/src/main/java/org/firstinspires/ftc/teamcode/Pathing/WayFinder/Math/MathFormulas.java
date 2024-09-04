package org.firstinspires.ftc.teamcode.Pathing.WayFinder.Math;

import org.firstinspires.ftc.teamcode.Pathing.WayFinder.Exceptions.NotAPolynomialException;

import java.util.Arrays;

import javax.annotation.Nullable;

public class MathFormulas {
    public static double TriangleArea(double base, double h) { return base * h / 2; }

    public static double RectangleArea(double length, double width) { return length * width; }

    /**solution 1 (+), solution 2 (-)*/
    @Nullable
    public static double[] QuadraticSolve(double a, double b, double c) {
        double delta = b * b - 4 * a * c;
        double solution1, solution2;

        if (delta > 0) {
            solution1 = ( -b + Math.sqrt(delta) ) / (2 * a);
            solution2 = ( -b - Math.sqrt(delta) ) / (2 * a);
        } else if (delta == 0) {
            solution1 = solution2 = -b / (2 * a);
        } else { return null; }

        return new double[] {solution1, solution2};
    }

    public static double toPower(double value, int power) {
        double resultedValue = 1;

        for (int i = 1; i <= power; i++)
            resultedValue *= value;

        return resultedValue;
    }

    public static double sign(double value) { return (value < 0) ? -1 : 1; }

    public static Coefficients findLinearFunction(Point point1, Point point2) throws NotAPolynomialException {
        Double a = new Double(( point1.y - point2.y ) / ( point1.x - point2.x) );
        Double b = new Double(point1.y - a * point1.x);

        return new Coefficients(Arrays.asList(a, b));
    }




}
