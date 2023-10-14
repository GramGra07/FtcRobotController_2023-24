package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.retractAirplane;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.shootAirplane;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.ggutil.pure.fromTutorial.com.company.Range;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

public class Drivers {
    public static final String[] driverControls = {"Chase", "Camden", "Kian", "Grady", "Michael"}, otherControls = driverControls;
    public static final int baseDriver = 0, baseOther = 1;//list integer of base driver and other controls
    public static int dIndex = baseDriver, oIndex = baseOther;//list integer of driver and other controls
    public static String currDriver = driverControls[dIndex], currOther = otherControls[oIndex];//list string of driver and other controls
    public static boolean fieldCentric;

    public static boolean optionsHigh1 = false, shareHigh1 = false, optionsHigh2 = false, shareHigh2 = false;
    public static boolean dDownHigh = false;

    public static void bindDriverButtons(OpMode myOpMode) {
        //"Chase", "Camden", "Kian", "Grady", "Michael"
        if (currDriver == driverControls[0]) {//Chase
            fieldCentric = false;
            //slowmode
            if (myOpMode.gamepad1.dpad_down && !dDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            }else if (myOpMode.gamepad1.dpad_down && !dDownHigh && slowModeIsOn){
                slowModeIsOn = false;
            }
            dDownHigh = myOpMode.gamepad1.dpad_down;
        }
        if (currDriver == driverControls[1]) {//Camden
            fieldCentric = false;
            //no slow mode
        }
        if (currDriver == driverControls[2]) {//Kian
            fieldCentric = true;
            //slowmode
            if (myOpMode.gamepad1.dpad_down && !dDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            }else if (myOpMode.gamepad1.dpad_down && !dDownHigh && slowModeIsOn){
                slowModeIsOn = false;
            }
            dDownHigh = myOpMode.gamepad1.dpad_down;
        }
        if (currDriver == driverControls[3]) {//Grady
            fieldCentric = true;
        }
        if (currDriver == driverControls[4]) {//Michael
            fieldCentric = false;
        }
    }

    public static void bindOtherButtons(OpMode myOpMode){
        //"Chase", "Camden", "Kian", "Grady", "Michael"
        if (!airplaneArmed && timer.seconds() > 80) {
            airplaneArmed = true;
            myOpMode.gamepad2.runRumbleEffect(cRE);
        }
        if (currOther == otherControls[0]) {//Chase
            if (myOpMode.gamepad2.circle && airplaneArmed){
                airplanePower = airplaneMax;
            }else{
                airplanePower = 0;
            }
            if (myOpMode.gamepad2.dpad_up){
                shootAirplane(airplaneServo);
            }else if (myOpMode.gamepad2.dpad_down){
                retractAirplane(airplaneServo);
            }
            if (myOpMode.gamepad2.dpad_up){
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down){
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.triangle){
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.cross){
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_y, liftMin, liftMax);
        }
        if (currOther == otherControls[1]) {//Camden
            if (myOpMode.gamepad2.cross && airplaneArmed){
                airplanePower = airplaneMax;
            }else{
                airplanePower = 0;
            }
            if (myOpMode.gamepad2.dpad_up){
                shootAirplane(airplaneServo);
            }else if (myOpMode.gamepad2.dpad_down){
                retractAirplane(airplaneServo);
            }
            if (myOpMode.gamepad2.right_bumper){
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper){
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down){
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_up){
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_y, liftMin, liftMax);
        }
        if (currOther == otherControls[2]) {//Kian
            if (myOpMode.gamepad2.circle && airplaneArmed){
                airplanePower = airplaneMax;
            }else{
                airplanePower = 0;
            }
            if (myOpMode.gamepad2.dpad_up){
                shootAirplane(airplaneServo);
            }else if (myOpMode.gamepad2.dpad_down){
                retractAirplane(airplaneServo);
            }
            if (myOpMode.gamepad2.right_bumper){
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper){
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_right){
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_left){
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_y, liftMin, liftMax);
        }
        if (currOther == otherControls[3]) {//Grady
            if (myOpMode.gamepad2.circle && airplaneArmed){
                airplanePower = airplaneMax;
            }else{
                airplanePower = 0;
            }
            if (myOpMode.gamepad2.dpad_up){
                shootAirplane(airplaneServo);
            }else if (myOpMode.gamepad2.dpad_down){
                retractAirplane(airplaneServo);
            }
            if (myOpMode.gamepad2.right_bumper){
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper){
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_right){
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.dpad_left){
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_y, liftMin, liftMax);
        }
        if (currOther == otherControls[4]) {//Michael
            if (myOpMode.gamepad2.square && airplaneArmed){
                airplanePower = airplaneMax;
            }else{
                airplanePower = 0;
            }
            if (myOpMode.gamepad2.dpad_up){
                shootAirplane(airplaneServo);
            }else if (myOpMode.gamepad2.dpad_down){
                retractAirplane(airplaneServo);
            }
            if (myOpMode.gamepad2.dpad_up){
                openClaw(claw1);
            }
            if (myOpMode.gamepad2.dpad_down){
                closeClaw(claw1);
            }
            if (myOpMode.gamepad2.left_bumper){
                openClaw(claw2);
            }
            if (myOpMode.gamepad2.right_bumper){
                closeClaw(claw2);
            }
            liftPower = Range.clip(myOpMode.gamepad2.left_stick_y, liftMin, liftMax);
        }
        Blink.selectLights(myOpMode);
    }


    public static void switchProfile(OpMode myOpMode) {
        //driver
        if (myOpMode.gamepad1.options && !optionsHigh1) {
            if (dIndex == driverControls.length - 1) {
                dIndex = 0;
            } else {
                dIndex++;
            }
            currDriver = driverControls[dIndex];
        }
        optionsHigh1 = myOpMode.gamepad1.options;
        if (myOpMode.gamepad1.share && !shareHigh1) {
            if (dIndex == 0) {
                dIndex = driverControls.length - 1;
            } else {
                dIndex--;
            }
            currDriver = driverControls[dIndex];
        }
        shareHigh1 = myOpMode.gamepad1.share;
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2) {
            if (oIndex == otherControls.length - 1) {
                oIndex = 0;
            } else {
                oIndex++;
            }
            currOther = otherControls[oIndex];
        }
        optionsHigh2 = myOpMode.gamepad2.options;
        if (myOpMode.gamepad2.share && !shareHigh2) {
            if (oIndex == 0) {
                oIndex = otherControls.length - 1;
            } else {
                oIndex--;
            }
            currOther = otherControls[oIndex];
        }
        shareHigh2 = myOpMode.gamepad2.share;
    }
}
