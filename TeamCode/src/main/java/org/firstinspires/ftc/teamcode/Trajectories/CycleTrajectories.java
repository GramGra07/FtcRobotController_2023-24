package org.firstinspires.ftc.teamcode.Trajectories;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.extendAndPlace;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config

public class CycleTrajectories {
    public static TrajectorySequence cycle(MecanumDrive drive, Pose2d spot, Pose2d startPose) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(36, spot.getY(), Math.toRadians(-90)))
                .lineToLinearHeading(spot)
                .addDisplacementMarker(() -> {
                    //grab pixel
                })
                .lineToLinearHeading(new Pose2d(36, spot.getY(), Math.toRadians(-90)))
                .lineToLinearHeading(startPose)
                .addDisplacementMarker(() -> {
                    extendAndPlace(drive);
                })
                .build();
    }
    public static TrajectorySequence quickLong(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(spot)
                .addDisplacementMarker(() -> {
                    //grab pixel
                })

                .build();
    }
}
