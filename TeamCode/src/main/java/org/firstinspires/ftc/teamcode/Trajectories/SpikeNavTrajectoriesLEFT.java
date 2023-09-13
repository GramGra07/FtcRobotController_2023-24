package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesLEFT {
    public static TrajectorySequence navToSpikeRightL(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(31)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addDisplacementMarker(()->{})
                .back(16)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftLRed(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(32)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(()->{})
                .back(6)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftLBlue(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(32)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(()->{})
                .back(6)
                .strafeLeft(12)
                .forward(15)
                .build();
    }
    public static TrajectorySequence navToSpikeCenterL(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(36)
                .strafeLeft(2)
                .addDisplacementMarker(()->{})
                .back(10)
                .strafeLeft(12)
                .build();
    }
}
