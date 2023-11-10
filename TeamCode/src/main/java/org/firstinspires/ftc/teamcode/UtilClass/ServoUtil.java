package org.firstinspires.ftc.teamcode.UtilClass;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@Config

public class ServoUtil {
    public static double position = 0;
    public final static double degree_mult = 0.00555555554;

    public static double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }
    public static int openClaw1 = 190;
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
    public static int closeClaw1 = 130;
    public static int closeClaw2 = 30;
    public static void closeClaw(Servo servo) {
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(closeClaw1));
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
        } else if (servo == HardwareConfig.claw2) {
            servo.setPosition(setServo(closeClaw2));
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
        }
    }

    public static int servoFlipBase = 25;
    public static int servoFlipFull = 157;
    public static int servoFlipFullHalf = 90;

    public static void flipServoBase(Servo servo) {
        servo.setPosition(setServo(servoFlipBase));
    }

    public static void flipServoFull(Servo servo) {
        servo.setPosition(setServo(servoFlipFull));
    }

    public static void flipServoFullHalf(Servo servo) {
        servo.setPosition(setServo(servoFlipFullHalf));
    }
}
