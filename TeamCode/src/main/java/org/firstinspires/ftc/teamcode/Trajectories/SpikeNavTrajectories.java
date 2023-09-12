package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectories {
    public static TrajectorySequence navToSpikeRight(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(10)
                .strafeLeft(10)
                .turn(Math.toRadians(90))
                .build();
    }
    public static TrajectorySequence navToSpikeLeft(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(10)
                .strafeRight(10)
                .turn(Math.toRadians(-90))
                .build();
    }
    public static TrajectorySequence navToSpikeCenter(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(10)
                .build();
    }
}
