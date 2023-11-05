package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesRIGHT {
    public static TrajectorySequence navToSpikeRightR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeRight(10)
                .forward(24)
                .addDisplacementMarker(() -> {
                })
                .back(16)
                .build();
    }

    public static TrajectorySequence navToSpikeLeftR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(31)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(() -> {
                })
                .back(6)
                .build();
    }

    public static TrajectorySequence navToSpikeCenterR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(36)
                .strafeLeft(2)
                .addDisplacementMarker(() -> {
                })
                .back(10)
                .build();
    }
}
