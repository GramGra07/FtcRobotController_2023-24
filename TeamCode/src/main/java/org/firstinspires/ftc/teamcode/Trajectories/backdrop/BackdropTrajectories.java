package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.raiseArm;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.PresetPose;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config
public class BackdropTrajectories {
    public static int endAngle = 0;
    public static int offset = 8;
    public static int startOffsetRed = 3;
    public static int startOffsetBlue = 2;
    public static int xOffset = 4;
    public static int backdropOffset = 6;
    public static Pose2d backRed = new Pose2d(56, -30, Math.toRadians(endAngle));
    public static Pose2d backBlue = new Pose2d(58, 38, Math.toRadians(endAngle));

    public static TrajectorySequence redShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .lineToLinearHeading(backRed)
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive, PathLong pathLong, boolean isCycling) {
        updatePose(drive);
        switch (pathLong) {
            case INSIDE:
                switch (autoRandomReliable) {
                    case mid:
                    case right:
                        int strafeRightOffset = 4;
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                                .lineToLinearHeading(new Pose2d(-52, -10, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))
                                .addDisplacementMarker(() -> {
                                    raiseArm(0, PresetPose.HIGH);
                                    ServoUtil.calculateFlipPose(30, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backRed.getX() - backdropOffset, backRed.getY() - strafeRightOffset, backRed.getHeading()), Math.toRadians(endAngle))
                                .forward(backdropOffset)
//                                .strafeRight(7)
                                .build();
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX() + 4, -10))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))
                                .addDisplacementMarker(() -> {
                                    raiseArm(0, PresetPose.HIGH);
                                    ServoUtil.calculateFlipPose(30, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backRed.getX() - backdropOffset, backRed.getY(), backRed.getHeading()), Math.toRadians(endAngle))
                                .forward(backdropOffset)
                                .build();
                }
            case OUTSIDE:
                int strafeOffset = 0; // - goes right, + goes left
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                        .addDisplacementMarker(() -> {
                            raiseArm(0, PresetPose.HIGH);
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(new Pose2d(backRed.getX() - backdropOffset, backRed.getY() - strafeOffset, backRed.getHeading()), Math.toRadians(endAngle))
                        .forward(backdropOffset)
//                        .strafeLeft(7)
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

    public static TrajectorySequence blueLong(MecanumDrive drive, PathLong pathLong, boolean isCycling) {
        updatePose(drive);
        switch (pathLong) {
            case INSIDE:
                switch (autoRandomReliable) {
                    case left:
                    case mid:
                        int strafeOffset = 5; // - goes right, + goes left
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 10))
                                .lineTo(new Vector2d(36, 30))
                                .addDisplacementMarker(() -> {
                                    raiseArm(0, PresetPose.HIGH);
                                    ServoUtil.calculateFlipPose(30, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backBlue.getX() - backdropOffset, backBlue.getY() + strafeOffset, backBlue.getHeading()), Math.toRadians(endAngle))
                                .forward(backdropOffset)
//                                .strafeLeft(9)
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX() - 4, 12))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(36, 12))
                                .lineTo(new Vector2d(36, 30))
                                .addDisplacementMarker(() -> {
                                    raiseArm(0, PresetPose.HIGH);
                                    ServoUtil.calculateFlipPose(30, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backBlue.getX() - backdropOffset, backBlue.getY(), backBlue.getHeading()), Math.toRadians(endAngle))
                                .forward(backdropOffset + 8)
                                .build();
                }
            case OUTSIDE:
                int strafeOffset = -4; // - goes right, + goes left
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(60, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                        .lineTo(new Vector2d(36, 30))
                        .addDisplacementMarker(() -> {
                            raiseArm(0, PresetPose.HIGH);
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .splineToLinearHeading(new Pose2d(backBlue.getX() - backdropOffset, backBlue.getY() - strafeOffset, backBlue.getHeading()), Math.toRadians(endAngle))
                        .forward(backdropOffset)
                        .build();
            default:
                return null;
        }
    }
}
