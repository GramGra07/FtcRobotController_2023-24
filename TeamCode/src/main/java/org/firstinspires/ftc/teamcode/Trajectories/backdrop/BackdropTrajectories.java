package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.Limits.autoExtension;
import static org.firstinspires.ftc.teamcode.Limits.autoRotation;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PotentPositions.autoPotent;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PotentPositions.potentiometerBase;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.rotationPower;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.encoderDrive;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.targetPositionPotent;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.targetPositionSlides;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StrafeOffsets;
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
    public static Pose2d backRed = new Pose2d(58, -32, Math.toRadians(endAngle));
    public static Pose2d backBlue = new Pose2d(58, 38, Math.toRadians(endAngle));

    public static TrajectorySequence redShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(()-> {
                    targetPositionSlides = autoExtension;
                    targetPositionPotent = autoRotation;
                })
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
                        int strafeRightOffset = StrafeOffsets.RLI_mr; //
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(-52, -10, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))
                                .addDisplacementMarker(() -> {
                                    targetPositionPotent = autoRotation;
                                    targetPositionSlides = autoExtension;
//                                    ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backRed.getX() - (backdropOffset * 2), backRed.getY() - strafeRightOffset, backRed.getHeading()), Math.toRadians(endAngle))
//                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                                .forward(backdropOffset + 4)
                                .build();
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX() + 4, -10))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))
                                .addDisplacementMarker(() -> {
                                    targetPositionPotent = autoRotation;
                                    targetPositionSlides = autoExtension;
                                })
                                .splineToLinearHeading(new Pose2d(backRed.getX() - (backdropOffset * 2), backRed.getY() - StrafeOffsets.RLI_l, backRed.getHeading()), Math.toRadians(endAngle))
//                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                                .forward(backdropOffset + 4)
                                .build();
                }
            case OUTSIDE:
                int strafeOffset = StrafeOffsets.RLO; // - goes right, + goes left
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                        .addDisplacementMarker(() -> {
                            targetPositionPotent = autoRotation;
                            targetPositionSlides = autoExtension;
//                            ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        })
                        .splineToLinearHeading(new Pose2d(backRed.getX() - (backdropOffset * 2), backRed.getY() - strafeOffset, backRed.getHeading()), Math.toRadians(endAngle))
//                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                        .forward(backdropOffset)
                        .build();
            default:
                return null;
        }
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(backBlue)
//                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
//                .back(1)
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive, PathLong pathLong, boolean isCycling) {
        updatePose(drive);
        switch (pathLong) {
            case INSIDE:
                switch (autoRandomReliable) {
                    case left:
                    case mid:
                        int strafeOffset = StrafeOffsets.BRI_ml; // - goes right, + goes left
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 10))
                                .lineTo(new Vector2d(36, 30))
                                .addDisplacementMarker(() -> {
                                    targetPositionPotent = autoRotation;
                                    targetPositionSlides = autoExtension;
//                                    ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backBlue.getX() - (backdropOffset * 2), backBlue.getY() + strafeOffset, backBlue.getHeading()), Math.toRadians(endAngle))
//                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                                .forward(backdropOffset + 4)
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX() - 4, 12))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(36, 12))
                                .lineTo(new Vector2d(36, 30))
                                .addDisplacementMarker(() -> {
                                    targetPositionPotent = autoRotation;
                                    targetPositionSlides = autoExtension;
//                                    ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                                })
                                .splineToLinearHeading(new Pose2d(backBlue.getX() - (backdropOffset * 2), backBlue.getY() - StrafeOffsets.BRI_r, backBlue.getHeading()), Math.toRadians(endAngle))
//                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                                .forward(backdropOffset + 8)
                                .build();
                }
            case OUTSIDE:
                int strafeOffset = StrafeOffsets.BRO; // - goes right, + goes left
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                        .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                        .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                        .lineTo(new Vector2d(36, 30))
                        .addDisplacementMarker(() -> {
                            targetPositionPotent = autoRotation;
                            targetPositionSlides = autoExtension;
//                            ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        })
                        .splineToLinearHeading(new Pose2d(backBlue.getX() - (backdropOffset * 2), backBlue.getY() - strafeOffset, backBlue.getHeading()), Math.toRadians(endAngle))
//                        .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo))
                        .forward(backdropOffset + 4)
                        .build();
            default:
                return null;
        }
    }
}
