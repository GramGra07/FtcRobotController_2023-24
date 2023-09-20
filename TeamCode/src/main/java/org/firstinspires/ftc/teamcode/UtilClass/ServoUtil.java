package org.firstinspires.ftc.teamcode.UtilClass;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

public class ServoUtil {
    public static int baseServo = 95;
    public static int clawMovement = 50;
    public static double position = 0;
    public final static double degree_mult = 0.00555555554;
    public static double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }
    public static void openClaw(Servo servo){
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(baseServo + clawMovement));
        }else if (servo == HardwareConfig.claw2){
            servo.setPosition(setServo(baseServo - clawMovement));
        }
    }
    public static void closeClaw(Servo servo){
        servo.setPosition(setServo(baseServo));
    }
}