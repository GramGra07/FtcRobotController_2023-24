package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1Possessed;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2Possessed;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.lastTimeOpen;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.potentiometer;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.timer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastAngle;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@Config

public class ServoUtil {
    public static int backClaw = 20;
    public static boolean useAutoClose = false;
    public static double autoCloseDist = 6;
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
            claw1Possessed = false;
        } else if (servo == HardwareConfig.claw2) {
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
            servo.setPosition(setServo(openClaw2));
            claw2Possessed = false;
        }
        lastTimeOpen = timer.seconds();
    }

    public static int closeClaw1 = 200;
    public static int closeClaw2 = 10;

    public static void closeClaw(Servo servo) {
        if (servo == HardwareConfig.claw1) {
            servo.setPosition(setServo(closeClaw1));
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
            claw1Possessed = true;
        } else if (servo == HardwareConfig.claw2) {
            servo.setPosition(setServo(closeClaw2));
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
            claw2Possessed = true;
        }
    }

    public static int servoFlipVal = 62;
    public static double hcalc = 96;
    public static double flipOffset = -30;

    public static void calculateFlipPose(int pose, Servo servo) {
        double theta = Sensors.getPotentVal(potentiometer) + flipOffset;
        PastAngle.pastAngleVal = theta;
        double sig = Math.ceil((-0.26 * theta) + hcalc) + (pose / 2);
        servo.setPosition(setServo(sig));
        servoFlipVal = (int) sig;
        lastSetVal = pose;
    }

    public static int lastSetVal;
    public static int releaseAirplane = 120;

    public static void releaseAirplane(Servo servo) {
        servo.setPosition(setServo(releaseAirplane));
    }

    public static int raiseAirplaneVal = 40;
    public static int airplaneReset = 30;

    public static void resetAirplane(Servo servo) {
        servo.setPosition(setServo(airplaneReset));
    }

    public static void raiseAirplane(Servo servo) {
        servo.setPosition(setServo(raiseAirplaneVal));
    }

    public static int downClawRigging = -20;
}
