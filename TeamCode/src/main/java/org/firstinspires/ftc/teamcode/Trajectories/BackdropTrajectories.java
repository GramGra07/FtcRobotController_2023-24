package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class BackdropTrajectories {
    public static TrajectorySequence redShort(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(46, -36, Math.toRadians(0)))
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(48, -36,Math.toRadians(0)))
                .build();
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeLeft(24)
                .forward(16)
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
                .build();
    }
}
