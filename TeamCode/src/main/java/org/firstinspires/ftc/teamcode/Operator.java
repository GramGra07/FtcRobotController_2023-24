package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Limits.flipperMax;
import static org.firstinspires.ftc.teamcode.Limits.flipperMin;
import static org.firstinspires.ftc.teamcode.Limits.liftMax;
import static org.firstinspires.ftc.teamcode.Limits.slideMax;
import static org.firstinspires.ftc.teamcode.Limits.slideMin;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoBase;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoFull;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoFullHalf;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.airplaneArmed;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.cRE;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.rotationPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.liftPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.extensionPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.timer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.ggutil.pure.fromTutorial.com.company.Range;

public class Operator extends Drivers {
    public static void bindOtherButtons(OpMode myOpMode) {
        //"Chase", "Camden", "Kian", "Grady", "Michael","Graden"
        if (!airplaneArmed && timer.seconds() > 80) {
            airplaneArmed = true;
            myOpMode.gamepad2.runRumbleEffect(cRE);
        }
        Blink.selectLights(myOpMode);
        if (currOther == otherControls[0]) {//Chase
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up) {
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.right_stick_y > 0) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                flipServoBase(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[1]) {//Camden
            if (myOpMode.gamepad2.right_bumper) {
                closeClaw(claw2);
            }
            if (myOpMode.gamepad2.left_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.right_trigger > 0) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.left_trigger > 0) {
                closeClaw(claw1);
            }
//            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(-myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.dpad_left) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.dpad_right) {
                flipServoBase(flipServo);
            } else if (myOpMode.gamepad2.dpad_down) {
                flipServoFullHalf(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[2]) {//Kian
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up) {
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.right_stick_y > 0) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                flipServoBase(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[3]) {//Grady
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up) {
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.right_stick_y > 0) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                flipServoBase(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[4]) {//Michael
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up) {
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.right_stick_y > 0) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                flipServoBase(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
        if (currOther == otherControls[5]) {//Graden
            if (myOpMode.gamepad2.right_bumper) {
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper) {
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down) {
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up) {
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_x, -liftMax, liftMax);
            extensionPower = Range.clip(myOpMode.gamepad2.left_stick_y, slideMin, slideMax);
            if (myOpMode.gamepad2.right_stick_y > 0) {
                flipServoFull(flipServo);
            } else if (myOpMode.gamepad2.right_stick_y < 0) {
                flipServoBase(flipServo);
            }
            rotationPower = Range.clip(myOpMode.gamepad2.right_stick_y, flipperMin, flipperMax);
        }
    }
}
