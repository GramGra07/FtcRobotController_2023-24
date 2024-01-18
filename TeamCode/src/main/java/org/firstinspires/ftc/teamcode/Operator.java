package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Limits.flipperMax;
import static org.firstinspires.ftc.teamcode.Limits.flipperMin;
import static org.firstinspires.ftc.teamcode.Limits.slideMax;
import static org.firstinspires.ftc.teamcode.Limits.slideMin;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.lastSetVal;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.useAutoClose;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime.useLoopTime;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.extensionPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.potentiometer;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.rotationPower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastPotent;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class Operator extends Drivers {
    public static boolean touchPressed = false;

    public static void bindOtherButtons(OpMode myOpMode, MecanumDrive drive) {
        //"Chase", "Camden", "Kian", "Grady", "Michael","Graden"
//        if (!airplaneArmed && timer.seconds() > 80) {
//            airplaneArmed = true;
//            myOpMode.gamepad2.runRumbleEffect(cRE);
//        }
//        Blink.selectLights(myOpMode);
        if (currOther == otherControls[1]) {//Camden
            if (!touchPressed && myOpMode.gamepad2.touchpad && useAutoClose) {
                useAutoClose = false;
                useLoopTime = false;
            } else if (!touchPressed && myOpMode.gamepad2.touchpad && !useAutoClose) {
                useAutoClose = true;
                useLoopTime = true;
            }
            touchPressed = myOpMode.gamepad2.touchpad;
//            if (drive.getPoseEstimate().getHeading()>-90 && drive.getPoseEstimate().getHeading()<90) {
            if (myOpMode.gamepad2.right_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw2);
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                openClaw(claw2);
            }
//            }else{
//                if (myOpMode.gamepad2.right_bumper) {
//                    closeClaw(claw2);
//                }
//                if (myOpMode.gamepad2.left_bumper) {
//                    closeClaw(claw1);
//                }
//                if (myOpMode.gamepad2.right_trigger > 0) {
//                    openClaw(claw2);
//                }
//                if (myOpMode.gamepad2.left_trigger > 0) {
//                    openClaw(claw1);
//                }
//            }

            extensionPower = Range.clip(-myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.dpad_left) {
                calculateFlipPose(30, flipServo);
            } else if (myOpMode.gamepad2.dpad_right) {
                calculateFlipPose(45, flipServo);
            } else if (myOpMode.gamepad2.dpad_up) {
                calculateFlipPose(70, flipServo);
            } else if (myOpMode.gamepad2.dpad_down) {
                calculateFlipPose(0, flipServo);
            }
            if (PastPotent.pastPotentVal != Sensors.getPotentVal(potentiometer)) {
                calculateFlipPose(lastSetVal, flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[3]) {//Grady
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                closeClaw(claw2);
            }

            extensionPower = Range.clip(-myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.dpad_down) {
                calculateFlipPose(0, flipServo);
            } else if (myOpMode.gamepad2.dpad_left) {
                calculateFlipPose(30, flipServo);
            }
            if (PastPotent.pastPotentVal != Sensors.getPotentVal(potentiometer)) {
                calculateFlipPose(lastSetVal, flipServo);
            }
            rotationPower = Range.clip(-myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[0]) {//Chase
        }
        if (currOther == otherControls[2]) {//Kian
        }
        if (currOther == otherControls[4]) {//Michael
        }
        if (currOther == otherControls[5]) {//Graden
            if (!touchPressed && myOpMode.gamepad2.touchpad && useAutoClose) {
                useAutoClose = false;
            } else if (!touchPressed && myOpMode.gamepad2.touchpad && !useAutoClose) {
                useAutoClose = true;
            }
            touchPressed = myOpMode.gamepad2.touchpad;

            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }

            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }

            if (myOpMode.gamepad2.left_trigger > 0) {
                closeClaw(claw2);
            }

            if (myOpMode.gamepad2.right_trigger > 0) {
                openClaw(claw2);
            }

            extensionPower = Range.clip(-myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.dpad_left) {
                calculateFlipPose(30, flipServo);
            } else if (myOpMode.gamepad2.dpad_right) {
                calculateFlipPose(45, flipServo);
            } else if (myOpMode.gamepad2.dpad_up) {
                calculateFlipPose(70, flipServo);
            } else if (myOpMode.gamepad2.dpad_down) {
                calculateFlipPose(0, flipServo);
            }

            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
            if (PastPotent.pastPotentVal != Sensors.getPotentVal(potentiometer)) {
                calculateFlipPose(lastSetVal, flipServo);
            }
        }
        if (currOther == otherControls[6]) { // Delaney
        }
        if (currOther == otherControls[7]) { // Child
            calculateFlipPose(30, flipServo);
        }
    }
}
