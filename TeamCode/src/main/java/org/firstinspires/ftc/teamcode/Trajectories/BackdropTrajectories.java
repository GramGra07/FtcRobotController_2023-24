package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class BackdropTrajectories {
    public static TrajectorySequence redShort(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .build();
    }
    public static TrajectorySequence redLong(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .build();
    }
    public static TrajectorySequence blueShort(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .build();
    }
    public static TrajectorySequence blueLong(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .build();
    }
}
