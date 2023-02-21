package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Constants {
    public static int turn = 77;
    public static double yMult = 24;
    public static double xMult = 10;
    public static double ovrCurrX = 0;
    public static double ovrCurrY = 0;
    public static double ovrTurn = 0;
    public static final double COUNTS_PER_INCH_Side_dead = -665.08;
    public static final double COUNTS_PER_INCH_Side = -100;

    public static final double COUNTS_PER_MOTOR_REV_arm = 28;
    public static final double DRIVE_GEAR_REDUCTION_arm = 40;
    public static final double WHEEL_DIAMETER_INCHES_arm = 1.102;     // For figuring circumference
    public static final double COUNTS_PER_INCH_arm = (COUNTS_PER_MOTOR_REV_arm * DRIVE_GEAR_REDUCTION_arm) /
            (WHEEL_DIAMETER_INCHES_arm * 3.1415);
    //wheels
    public static final double COUNTS_PER_MOTOR_REV = 28;
    public static final double WHEEL_DIAMETER_MM = 96;
    public static final double DRIVE_GEAR_REDUCTION = 15;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double COUNTS_PER_MOTOR_REV_dead = 8192;
    public static final double WHEEL_DIAMETER_MM_dead = 96;
    public static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);
    public final int baseArmPosition = 0;
    public static final int armLimit = 4250;//declaring the armLimit variable
    public final int baseArm = 0;//declaring the baseArm variable
    public static final int lowPoleVal = 1570;//should be about 1/3 of arm limit
    public static final int midPoleVal = 290;//should be about 2/3 of arm limit
    public static final int fiveTallConeVal = 300;
    public static final int topPoleVal = armLimit;//should be close to armLimit


    //tape measure
    public static double tapePower;
    public static double tapeMeasureDiameter = 7.5;
    public static int tapeMeasureLength = 15 * 12;
    public static double countsPerInchTape = 10;
    public static double tickPerTapeMeasure = countsPerInchTape * tapeMeasureLength;
    public static int tmPose = 68;
}
