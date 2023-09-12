package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class ShiftTrajectories {
    public static TrajectorySequence shiftLeft(SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeLeft(10)
                .build();
    }
    public static TrajectorySequence shiftRight(SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeRight(10)
                .build();
    }
}
