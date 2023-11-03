package org.firstinspires.ftc.teamcode.UtilClass;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@Config

public class ServoUtil {
    public static int baseServo = 95;
    public static int clawMovement = 50;
    public static int offset = 5;
    public static double position = 0;
    public final static double degree_mult = 0.00555555554;

    public static double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public static void openClaw(Servo servo) {
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(baseServo + clawMovement));
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
        } else if (servo == HardwareConfig.claw2) {
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
            servo.setPosition(setServo(baseServo + clawMovement+offset));
        }
    }

    public static void closeClaw(Servo servo) {
        servo.setPosition(setServo(baseServo));
        if (servo == HardwareConfig.claw1) {
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
        } else if (servo == HardwareConfig.claw2) {
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
