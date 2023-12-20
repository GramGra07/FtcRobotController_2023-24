package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.potentiometer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastPotent;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@Config

public class ServoUtil {
    public static double position = 0;
    public final static double degree_mult = 0.00555555554;

    public static double setServo(double degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public static double convertToDegrees(double pose) {
        return pose / degree_mult;
    }

    public static int openClaw1 = 100;
    public static int openClaw2 = 90;

    public static void openClaw(Servo servo) {
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(openClaw1));
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
        } else if (servo == HardwareConfig.claw2) {
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
            servo.setPosition(setServo(openClaw2));
        }
    }

    public static int closeClaw1 = 155;
    public static int closeClaw2 = 45;

    public static void closeClaw(Servo servo) {
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(closeClaw1));
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
        } else if (servo == HardwareConfig.claw2) {
            servo.setPosition(setServo(closeClaw2));
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
        }
    }

    public static int servoFlipBase = 87;
    public static int servoFlipFull = 62;
    public static int servoFlipFullHalf = 75;
    public static int servoFlipVal = servoFlipFull;

    public static void flipServoBase(Servo servo) {
        servo.setPosition(setServo(servoFlipBase));
//        lastSetVal = servoFlipBase;
    }

    public static void flipServoFull(Servo servo) {
        servo.setPosition(setServo(servoFlipFull));
//        lastSetVal = servoFlipFull;
    }

    public static void flipServoFullHalf(Servo servo) {
        servo.setPosition(setServo(servoFlipFullHalf));
//        lastSetVal = servoFlipFullHalf;
    }

    public static double hcalc = 84;
    public static void calculateFlipPose(int pose, Servo servo) {
        double theta = Sensors.getPotentVal(potentiometer);
        PastPotent.pastPotentVal = theta;
        double sig = Math.ceil((-0.26 * theta) + hcalc) + (pose / 2);
        servo.setPosition(setServo(sig));
        servoFlipVal = (int) sig;
        lastSetVal = pose;
    }

    public static int lastSetVal;
    public static int releaseAirplane = 30;
    public static void releaseAirplane(Servo servo){
        servo.setPosition(setServo(releaseAirplane));
    }
}
