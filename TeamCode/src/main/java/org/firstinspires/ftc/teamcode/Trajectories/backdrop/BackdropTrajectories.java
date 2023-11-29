package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.blueRotate;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.extendAndPlace;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config
public class BackdropTrajectories {
    public static int endAngle = 0;

    public static TrajectorySequence redShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(autoHardware::dropAndRaise)
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(endAngle)))
                .addDisplacementMarker(() -> {
                    extendAndPlace(drive);
                })
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(autoHardware::dropAndRaise)
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(endAngle)))
                .addDisplacementMarker(() -> {
                    extendAndPlace(drive);
                })
                .build();
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(autoHardware::dropAndRaise)
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(endAngle)))
                .addDisplacementMarker(() -> {
                    extendAndPlace(drive);
                })
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(blueRotate + 90)))
                .forward(56)
                .addDisplacementMarker(autoHardware::dropAndRaise)
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(endAngle)))
                .addDisplacementMarker(() -> {
                    extendAndPlace(drive);
                })
                .build();
    }
}
