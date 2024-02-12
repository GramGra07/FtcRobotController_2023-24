package org.firstinspires.ftc.teamcode.Trajectories;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class spikeNavTraj {
    public static int mult = 1;

    public static TrajectorySequence midPiNav(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    if (StartPose.alliance == Alliance.RED) {
                        mult = -1;
                    }
                })
                .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - (25 * mult), START_POSE.getHeading()), START_POSE.getHeading())
                .build();
    }

    public static int rotate = -50;

    public static TrajectorySequence fwdLeft(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    if (StartPose.alliance == Alliance.RED) {
                        mult = -1;
                        rotate = -rotate;
                    }
                })
                .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - (22 * mult), START_POSE.getHeading() - rotate), START_POSE.getHeading() - rotate)
                .build();
    }

    public static TrajectorySequence fwdRight(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    rotate = 50;
                    if (StartPose.alliance == Alliance.RED) {
                        mult = -1;
                        rotate = -rotate;
                    }
                })
                .splineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - (22 * mult), START_POSE.getHeading() - rotate), START_POSE.getHeading() - rotate)
                .build();
    }
}
