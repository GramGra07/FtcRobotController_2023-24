package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getCycleSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.navToBackdrop_Place;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.pickFromSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class autoPatterns {
    // contains different auto patterns for different tasks
    public static void place1Pixel(MecanumDrive drive) {
        ServoUtil.calculateFlipPose(0,flipServo);
        SpikeNav(drive);
    }

    // does two pixel and then goes to the end pose
    public static void pixelPark(MecanumDrive drive) {
        place1Pixel(drive);
        navToBackdrop_Place(drive,true);
        goToGenericEndPose(drive);
    }

    // will end it on different sides depending on the alliance
    public static void goToGenericEndPose(MecanumDrive drive) {
        switch (StartPose.alliance) {
            case RED:
                drive.followTrajectorySequence(endPose.goToEndPose(endPose.endPoseRightRed, drive));
                break;
            case BLUE:
                drive.followTrajectorySequence(endPose.goToEndPose(endPose.endPoseLeftBlue, drive));
                break;
        }
    }

    // method to grab the pixel early on the long side and only if on long side
    public static void grabPixelLongSide(MecanumDrive drive) {
        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
            // long side
            pickFromSpot(drive);
            updatePose(drive);
        }
    }

    // for long side
    public static void place2Cycle(MecanumDrive drive) {
        place1Pixel(drive);
        grabPixelLongSide(drive);
        navToBackdrop_Place(drive,false);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        updatePose(drive);
    }

    // method to cycle in autonomous
    public static void cycle(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) { // simple for loop to cycle i times
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, drive.getPoseEstimate()));
            } else {
                //long side
            }
        }
    }
}
