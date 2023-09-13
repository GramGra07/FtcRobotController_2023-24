package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesRIGHT {
    public static TrajectorySequence navToSpikeRightR(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeRight(10)
                .forward(24)
                .addDisplacementMarker(()->{})
                .back(8)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftR(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(31)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(()->{})
                .back(6)
                .build();
    }
    public static TrajectorySequence navToSpikeCenterR(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(36)
                .strafeRight(2)
                .addDisplacementMarker(()->{})
                .back(10)
                .build();
    }
}
