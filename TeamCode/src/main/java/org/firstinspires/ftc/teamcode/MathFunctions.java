package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MathFunctions {

    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;// = 100/180

    public double calcServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public static boolean inRange(double value, double min, double max) {
        return (value >= min && value <= max);
    }

    public double averageOf(double[] values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    public static int getQuadrant(Pose2d pose) {
        int x = (int) pose.getX();
        int y = (int) pose.getY();
        if (x == 0) {
            x += 0.1; // compensates for divide by 0 error
        }
        if (y == 0) {
            y += 0.1; // compensates for divide by 0 error
        }
        double xSign = (double) x / Math.abs(x); // gets sign of x
        double ySign = (double) y / Math.abs(y); // gets sign of y
        boolean xIsPositive = (xSign == 1); // checks if x is positive
        boolean yIsPositive = (ySign == 1); // checks if y is positive
        if (xIsPositive && !yIsPositive) {
            return 1;
        } else if (xIsPositive && yIsPositive) {
            return 2;
        } else if (!xIsPositive && !yIsPositive) {
            return 3;
        } else if (!xIsPositive && yIsPositive) {
            return 4;
        } else {
            return 0;
        }
    }

    public static int threeFourths(int amount) {
        return (amount / 4) * 3;
    }
}
