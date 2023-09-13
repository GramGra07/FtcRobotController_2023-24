package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectories {
    public static TrajectorySequence navToSpikeRightRIGHT(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .strafeRight(10)
                .forward(24)
                .back(8)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftRIGHT(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(31)
                .turn(Math.toRadians(90))
                .forward(5)
                .back(6)
                .build();
    }
    public static TrajectorySequence navToSpikeCenterRIGHT(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(36)
                .strafeRight(2)
                .back(10)
                .build();
    }
    public static TrajectorySequence navToSpikeRightLEFT(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(31)
                .turn(Math.toRadians(-90))
                .forward(5)
                .back(6)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftLRed(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(32)
                .turn(Math.toRadians(90))
                .forward(5)
                .back(6)
                .build();
    }
    public static TrajectorySequence navToSpikeLeftLBlue(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(32)
                .turn(Math.toRadians(90))
                .forward(5)
                .back(6)
                .strafeLeft(12)
                .forward(15)
                .build();
    }
    public static TrajectorySequence navToSpikeCenterLEFT(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(36)
                .strafeLeft(2)
                .back(10)
                .build();
    }
}
