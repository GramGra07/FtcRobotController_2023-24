package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.raiseArm;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config
public class BackdropTrajectories {
    public static int endAngle = 0;
    public static int offset = 8;
    public static int startOffset = 3;
    public static int xOffset = 4;
    public static Pose2d backRed = new Pose2d(56, -30, Math.toRadians(endAngle));
    public static Pose2d backBlue = new Pose2d(58, 38, Math.toRadians(endAngle));

    public static TrajectorySequence redShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .lineToLinearHeading(backRed)
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive, PathLong pathLong) {
        updatePose(drive);
        switch (pathLong) {
            case INSIDE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(endAngle)))
                        .lineTo(new Vector2d(36, -12))
                        .addDisplacementMarker(() -> {
                            raiseArm();
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(backRed, Math.toRadians(endAngle))
                        .build();
            case OUTSIDE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffset, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffset, Math.toRadians(endAngle)))
                        .addDisplacementMarker(() -> {
                            raiseArm();
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(backRed, Math.toRadians(endAngle))
                        .strafeLeft(8)
                        .build();
            default:
                return null;
        }
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .lineToLinearHeading(backBlue)
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive, PathLong pathLong) {
        updatePose(drive);
        switch (pathLong) {
            case INSIDE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(endAngle)))
                        .lineTo(new Vector2d(36, 12))
                        .addDisplacementMarker(() -> {
                            raiseArm();
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(backBlue, Math.toRadians(endAngle))
                        .build();
            case OUTSIDE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffset, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffset, Math.toRadians(endAngle)))
                        .addDisplacementMarker(() -> {
                            raiseArm();
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(backBlue, Math.toRadians(endAngle))
                        .build();
            default:
                return null;
        }
    }
}
