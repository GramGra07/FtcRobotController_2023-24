package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.slowModeIsOn;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

public class Drivers {
    public static final String[] driverControls = {"Chase", "Camden", "Kian", "Grady", "Michael", "Graden", "Delaney", "Child"}, otherControls = driverControls;
    public static final int baseDriver = 0, baseOther = 1;//list integer of base driver and other controls
    public static int dIndex = baseDriver, oIndex = baseOther;//list integer of driver and other controls
    public static String currDriver = driverControls[dIndex], currOther = otherControls[oIndex];//list string of driver and other controls
    public static boolean fieldCentric;

    public static boolean optionsHigh1 = false, shareHigh1 = false, optionsHigh2 = false, shareHigh2 = false;
    public static boolean circleDownHigh = false;

    public static void bindDriverButtons(OpMode myOpMode, MecanumDrive drive) {
        //"Chase", "Camden", "Kian", "Grady", "Michael","Graden", "Delaney", "Child"
        if (currDriver == driverControls[0]) {//Chase
            fieldCentric = false;
            //slowmode
            if (myOpMode.gamepad1.circle && !circleDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !circleDownHigh && slowModeIsOn) {
                slowModeIsOn = false;
            }
            circleDownHigh = myOpMode.gamepad1.circle;
            if (myOpMode.gamepad1.right_bumper) {
                slowModeIsOn = false;
                IsBusy.isAutoInTeleop = true;
                drive.update();
                if (StartPose.alliance == Alliance.RED) {
                    drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(-50, 50, Math.toRadians(135)), Math.toRadians(135))
                            .build());
                } else {
                    drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(-135)), Math.toRadians(-135))
                            .build());
                }
            }
            if (myOpMode.gamepad1.dpad_up) {
                slowModeIsOn = false;
                IsBusy.isAutoInTeleop = true;
                drive.turnAsync(Angle.normDelta(Math.toRadians(0) - drive.getPoseEstimate().getHeading()));
            }
            if (myOpMode.gamepad1.dpad_left) {
                slowModeIsOn = false;
                IsBusy.isAutoInTeleop = true;
                if (StartPose.alliance == Alliance.RED) {
                    drive.turnAsync(Angle.normDelta(Math.toRadians(135) - drive.getPoseEstimate().getHeading()));
                } else {
                    drive.turnAsync(Angle.normDelta(Math.toRadians(-135) - drive.getPoseEstimate().getHeading()));
                }
            }
            if (!drive.isBusy()) {
                IsBusy.isAutoInTeleop = false;
            }
            if (myOpMode.gamepad1.cross) {
                IsBusy.isAutoInTeleop = true;
                drive.breakFollowing();
            }
        }
        if (currDriver == driverControls[1]) {//Camden
            fieldCentric = false;
            //no slow mode
        }
        if (currDriver == driverControls[2]) {//Kian
            fieldCentric = true;
            //slowmode
            if (myOpMode.gamepad1.dpad_down && !circleDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.dpad_down && !circleDownHigh && slowModeIsOn) {
                slowModeIsOn = false;
            }
            circleDownHigh = myOpMode.gamepad1.dpad_down;
        }
        if (currDriver == driverControls[3]) {//Grady
            fieldCentric = true;
        }
        if (currDriver == driverControls[4]) {//Michael
            fieldCentric = false;
        }
        if (currDriver == driverControls[5]) {//Graden
            fieldCentric = false;
        }
        if (currDriver == driverControls[6]) {//Delaney
            fieldCentric = false;
            //slowmode
            if (myOpMode.gamepad1.circle && !circleDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            } else if (myOpMode.gamepad1.circle && !circleDownHigh && slowModeIsOn) {
                slowModeIsOn = false;
            }
            circleDownHigh = myOpMode.gamepad1.circle;
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
        if (currDriver == driverControls[7]) {
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
