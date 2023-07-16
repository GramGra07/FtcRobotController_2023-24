package org.firstinspires.ftc.teamcode;

public class MathFunctions {
    public boolean inRange(double value, double min, double max) {
        return (value >= min && value <= max);
    }

    public double averageOf(double[] values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }
}
