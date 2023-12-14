package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getCycleSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.pickFromSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class autoPatterns {
    public static void place1Pixel(MecanumDrive drive) {
        ServoUtil.calculateFlipPose(60, flipServo);
        SpikeNav(drive);
    }

    public static void pixelPark(MecanumDrive drive) {
        place1Pixel(drive);
        navToBackdrop(drive,true);
        goToGenericEndPose(drive);
    }

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

    public static void grabPixelLongSide(MecanumDrive drive) {
        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
            // long side
            pickFromSpot(drive);
            updatePose(drive);
        }
    }

    public static void place2Cycle(MecanumDrive drive) {
        place1Pixel(drive);
        grabPixelLongSide(drive);
        navToBackdrop(drive,false);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        updatePose(drive);
    }

    public static void cycle(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) {
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, drive.getPoseEstimate()));
            } else {
                //long side
            }
        }
    }
}
