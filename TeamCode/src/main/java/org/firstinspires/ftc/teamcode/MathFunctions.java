package org.firstinspires.ftc.teamcode;

public class MathFunctions {

    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;// = 100/180

    public double calcServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

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
