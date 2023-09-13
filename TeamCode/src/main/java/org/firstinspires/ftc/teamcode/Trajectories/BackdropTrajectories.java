package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class BackdropTrajectories {
    public static TrajectorySequence redShort(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(46,-36,Math.toRadians(0)))
                .build();
    }
    public static TrajectorySequence redLong(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(-36, -58), Math.toRadians(0))
                .lineTo(new Vector2d(36,-58))
                .lineToLinearHeading(new Pose2d(46,-36,Math.toRadians(0)))
                .build();
    }
    public static TrajectorySequence blueShort(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineToLinearHeading(new Pose2d(46,36), Math.toRadians(0))
                .build();
    }
    public static TrajectorySequence blueLong(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(-36, 58), Math.toRadians(0))
                .lineTo(new Vector2d(36,58))
                .lineToLinearHeading(new Pose2d(46,36,Math.toRadians(0)))
                .build();
    }
}
