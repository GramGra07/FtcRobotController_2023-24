package org.firstinspires.ftc.teamcode.Trajectories;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class spikeNavTraj {

    public static TrajectorySequence midPiNav(MecanumDrive drive) {
        if (StartPose.alliance == Alliance.RED) {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX() + 8, START_POSE.getY() + (25), START_POSE.getHeading()), START_POSE.getHeading())
                    .build();
        } else {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX() + 8, START_POSE.getY() - (25), START_POSE.getHeading()), START_POSE.getHeading())
                    .build();
        }
    }

    public static int rotate = 70;

    public static TrajectorySequence fwdLeft(MecanumDrive drive) {
        if (StartPose.alliance == Alliance.BLUE) {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - (23), START_POSE.getHeading() + rotate - Math.toRadians(10)), START_POSE.getHeading() + rotate - Math.toRadians(10))
                    .build();
        } else {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() + (22), START_POSE.getHeading() + rotate - Math.toRadians(10)), START_POSE.getHeading() + rotate - Math.toRadians(10))
                    .build();
        }
    }

    public static TrajectorySequence fwdRight(MecanumDrive drive) {
        if (StartPose.alliance == Alliance.RED) {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() + (23), START_POSE.getHeading() - rotate), START_POSE.getHeading() - rotate)
                    .build();
        } else {
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - (23), START_POSE.getHeading() - rotate), START_POSE.getHeading() - rotate)
                    .build();
        }
    }
}
