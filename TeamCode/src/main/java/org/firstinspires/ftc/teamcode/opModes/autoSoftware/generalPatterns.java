package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.backdropOffset;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.blueMidOff;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.endAngle;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.forwardOffset;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetBlue;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetRed;
import static org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.fwdLeft;
import static org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.fwdRight;
import static org.firstinspires.ftc.teamcode.Trajectories.spikeNavTraj.midPiNav;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.startDist;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.Placement;
import org.firstinspires.ftc.teamcode.Enums.StartDist;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class generalPatterns {
    // method to go to the backdrop
    public static void navToBackdrop_Place(MecanumDrive drive, PathLong pathLong, boolean isCycling) {
        if (!isCycling) {
            switch (StartPose.alliance) {
                case RED:
                    switch (StartPose.side) {
                        case LEFT:
                            drive.followTrajectorySequenceAsync(BackdropTrajectories.redLong(drive, pathLong));
                            break;
                        case RIGHT:
                            drive.followTrajectorySequenceAsync(BackdropTrajectories.redShort(drive));
                            break;
                    }
                    break;
                case BLUE:
                    switch (StartPose.side) {
                        case LEFT:
                            drive.followTrajectorySequenceAsync(BackdropTrajectories.blueShort(drive));
                            break;
                        case RIGHT:
                            drive.followTrajectorySequenceAsync(BackdropTrajectories.blueLong(drive, pathLong));
                            break;
                    }
                    break;
            }
        } else {
            if (pathLong == PathLong.INSIDE) {
                switch (StartPose.alliance) {
                    case RED:
                        drive.followTrajectorySequenceAsync(BackdropTrajectories.redLong(drive, pathLong));
                        break;
                    case BLUE:
                        drive.followTrajectorySequenceAsync(BackdropTrajectories.blueLong(drive, pathLong));
                        break;
                }
            } else {
                int baseX;
                int baseY;
                switch (StartPose.alliance) {
                    case RED:
                        baseX = 58 + forwardOffset - backdropOffset;
                        baseY = -32 - 2;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-36, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() + startOffsetRed + 1, Math.toRadians(endAngle)))
                                .addSpatialMarker(new Vector2d(-24, -36), () ->
                                        calculateFlipPose(30, flipServo))
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build());
                        break;
                    case BLUE:
                        baseX = 58 + forwardOffset - backdropOffset;
                        baseY = 38 + blueMidOff;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-36, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue - 1, Math.toRadians(endAngle)))
                                .addSpatialMarker(new Vector2d(-24, -36), () ->
                                        calculateFlipPose(30, flipServo))
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build());
                        break;
                }
            }
        }
    }

    // drive and place first pixel
    public static void SpikeNav(MecanumDrive drive, PathLong pathLong) {
        Pose2d farSwingPoseRED = new Pose2d(-54, -22, Math.toRadians(0));
        Pose2d farSwingPoseBLUE = new Pose2d(farSwingPoseRED.getX() + 2, -farSwingPoseRED.getY(), farSwingPoseRED.getHeading());
        Placement placement;
        boolean RedRight = (startDist == StartDist.SHORT_SIDE && StartPose.alliance == Alliance.RED);
        boolean BlueLeft = (startDist == StartDist.SHORT_SIDE && StartPose.alliance == Alliance.BLUE);
        boolean BlueRight = (startDist == StartDist.LONG_SIDE && StartPose.alliance == Alliance.BLUE);
        boolean RedLeft = (startDist == StartDist.LONG_SIDE && StartPose.alliance == Alliance.RED);
        if (RedRight) placement = Placement.RED_RIGHT;
        else if (BlueLeft) placement = Placement.BLUE_LEFT;
        else if (BlueRight) placement = Placement.BLUE_RIGHT;
        else if (RedLeft) placement = Placement.RED_LEFT;
        else placement = Placement.RED_RIGHT;
        switch (autoHardware.autonomousRandom) {
            case left:
                switch (placement) {
                    case BLUE_RIGHT:
                    case RED_RIGHT:
                    case RED_LEFT:
                        drive.followTrajectorySequenceAsync(fwdLeft(drive));
                        break;
                    case BLUE_LEFT:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(START_POSE.getX() + 13, START_POSE.getY() - 20, START_POSE.getHeading()), START_POSE.getHeading())
                                .build()
                        );
                        break;
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.left;
                break;
            case mid:
                switch (placement) {
                    case RED_RIGHT:
                    case BLUE_LEFT:
                        drive.followTrajectorySequenceAsync(midPiNav(drive));
                        break;
                    case BLUE_RIGHT:
                        switch (pathLong) {
                            case NONE:
                            case OUTSIDE:
                                drive.followTrajectorySequenceAsync(midPiNav(drive));
                                break;
                            case INSIDE:
                                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(farSwingPoseBLUE, farSwingPoseBLUE.getHeading())
                                        .build()
                                );
                                break;
                        }
                        break;
                    case RED_LEFT:
                        switch (pathLong) {
                            case NONE:
                            case OUTSIDE:
                                drive.followTrajectorySequenceAsync(midPiNav(drive));
                                break;
                            case INSIDE:
                                drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(farSwingPoseRED, farSwingPoseRED.getHeading())
                                        .build()
                                );
                                break;
                        }
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.mid;
                break;
            case right:
                switch (placement) {
                    case BLUE_RIGHT:
                    case RED_LEFT:
                    case BLUE_LEFT:
                        drive.followTrajectorySequenceAsync(fwdRight(drive));
                        break;
                    case RED_RIGHT:
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(START_POSE.getX() + 18, START_POSE.getY() + 18, START_POSE.getHeading()), START_POSE.getHeading())
                                .build()
                        );
                        break;
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.right;
                break;
        }
    }
}
