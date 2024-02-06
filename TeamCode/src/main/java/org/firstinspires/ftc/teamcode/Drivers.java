package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriverIndex.dIndex;
import static org.firstinspires.ftc.teamcode.DriverIndex.oIndex;
import static org.firstinspires.ftc.teamcode.Limits.liftMax;
import static org.firstinspires.ftc.teamcode.Limits.liftMin;
import static org.firstinspires.ftc.teamcode.UtilClass.DriverAid.doDriverAid;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.resetAirplane;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.airplaneServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.extensionPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.liftPower;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.slowModeIsOn;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

public class Drivers {
    public static final String[] driverControls = {"Chase", "Camden", "Kian", "Grady", "Michael", "Graden", "Delaney", "Child"}, otherControls = driverControls;
    public static final int baseDriver = 0, baseOther = 1;//list integer of base driver and other controls
    public static String currDriver = driverControls[dIndex], currOther = otherControls[oIndex];//list string of driver and other controls
    public static boolean fieldCentric;
    public static boolean liftConnected = false;
    public static boolean optionsHigh1 = false, shareHigh1 = false, optionsHigh2 = false, shareHigh2 = false;
    public static boolean slowModeButtonDown = false, planeButtonDown = false, planeReleased = true;

    public static void bindDriverButtons(OpMode myOpMode, MecanumDrive drive) {
        //"Chase", "Camden", "Kian", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver == driverControls[0]) {//Chase
            fieldCentric = false;
            //slowmode
//            gamepad1.wasJustReleased(GamepadKeys.Button.B);
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && slowModeIsOn) {
                slowModeIsOn = false;
            }
            slowModeButtonDown = myOpMode.gamepad1.circle;
//            gamepad1.wasJustReleased(GamepadKeys.Button.Y);
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !planeReleased) {
                ServoUtil.releaseAirplane(airplaneServo);
                planeReleased = true;
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && planeReleased) {
                resetAirplane(airplaneServo);
                planeReleased = false;
            }
            planeButtonDown = myOpMode.gamepad1.triangle;
//            gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            // gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (myOpMode.gamepad1.right_trigger > 0) {
                liftPower = -liftMax;
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                liftPower = liftMax;
            } else {
                liftPower = 0;
            }
            if (liftConnected) {
                extensionPower = liftPower;
            }
            // using ftc lib
//            gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_UP);
//            gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT);
//            gamepad1.getButton(GamepadKeys.Button.A);
            doDriverAid(drive, myOpMode.gamepad1.right_bumper, myOpMode.gamepad1.dpad_up, myOpMode.gamepad1.dpad_right, myOpMode.gamepad1.cross);
        }
        if (currDriver == driverControls[3]) {//Grady
            fieldCentric = true;
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && slowModeIsOn) {
                slowModeIsOn = false;
            }
            slowModeButtonDown = myOpMode.gamepad1.circle;
            if (myOpMode.gamepad1.triangle && !planeButtonDown && !planeReleased) {
                ServoUtil.releaseAirplane(airplaneServo);
                planeReleased = true;
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && planeReleased) {
                airplaneServo.setPosition(ServoUtil.setServo(0));
                planeReleased = false;
            }
            planeButtonDown = myOpMode.gamepad1.triangle;
            if (myOpMode.gamepad1.right_trigger > 0) {
                liftPower = liftMax;
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                liftPower = -liftMax;
            } else {
                liftPower = 0;
            }
        }
        if (currDriver == driverControls[1]) {//Camden
        }
        if (currDriver == driverControls[2]) {//Kian
        }
        if (currDriver == driverControls[4]) {//Michael
        }
        if (currDriver == driverControls[5]) {//Graden
            fieldCentric = true;
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && slowModeIsOn) {
                slowModeIsOn = false;
            }
            slowModeButtonDown = myOpMode.gamepad1.circle;

            if (myOpMode.gamepad1.triangle && !planeButtonDown && !planeReleased) {
                ServoUtil.releaseAirplane(airplaneServo);
                planeReleased = true;
            } else if (myOpMode.gamepad1.triangle && !planeButtonDown && planeReleased) {
                resetAirplane(airplaneServo);
                planeReleased = false;
            }
            planeButtonDown = myOpMode.gamepad1.triangle;

            if (myOpMode.gamepad1.right_trigger > 0) {
                liftPower = liftMax;
            } else if (myOpMode.gamepad1.left_trigger > 0) {
                liftPower = -liftMax;
            } else {
                liftPower = 0;
            }

            doDriverAid(drive, myOpMode.gamepad1.right_bumper, myOpMode.gamepad1.dpad_up, myOpMode.gamepad1.dpad_right, myOpMode.gamepad1.cross);
        }
        if (currDriver == driverControls[6]) {//Delaney
            fieldCentric = false;
            //slowmode
            if (myOpMode.gamepad1.circle && !slowModeButtonDown && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !slowModeButtonDown && slowModeIsOn) {
                slowModeIsOn = false;
            }
            slowModeButtonDown = myOpMode.gamepad1.circle;
            if (myOpMode.gamepad1.triangle) {
                slowModeIsOn = false;
                IsBusy.isAutoInTeleop = true;
                drive.update();
                PoseStorage.currentPose = drive.getPoseEstimate();
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                        .splineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + 5, drive.getPoseEstimate().getY() + 10, (Angle.normDelta(Math.toRadians(135) - drive.getPoseEstimate().getHeading()))), Angle.normDelta(Math.toRadians(135) - drive.getPoseEstimate().getHeading()))
                        .addDisplacementMarker(() -> ServoUtil.openClaw(claw1))
                        .addDisplacementMarker(() -> ServoUtil.openClaw(claw2))
                        .splineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() - 5, drive.getPoseEstimate().getY() - 10, (Angle.normDelta(Math.toRadians(45) - drive.getPoseEstimate().getHeading()))), Angle.normDelta(Math.toRadians(135) - drive.getPoseEstimate().getHeading()))
                        .addDisplacementMarker(() -> ServoUtil.closeClaw(claw1))
                        .addDisplacementMarker(() -> ServoUtil.closeClaw(claw2))
                        .splineToLinearHeading(PoseStorage.currentPose, PoseStorage.currentPose.getHeading())
                        .build());
            }
            if (myOpMode.gamepad1.cross) {
                IsBusy.isAutoInTeleop = true;
                drive.breakFollowing();
            }
            if (!drive.isBusy()) {
                IsBusy.isAutoInTeleop = false;
            }
        }
        if (currDriver == driverControls[7]) {//Child
            fieldCentric = false;
            slowModeIsOn = true;
        }
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

        if (currDriver == driverControls[7]) {
            currOther = driverControls[7];
        }
    }
}
