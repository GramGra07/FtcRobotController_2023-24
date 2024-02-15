package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories.leftShift;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories.rightShift;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions;
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
    public static int blueMidOff = 0;
    public static int redMidOff = 0;
    public static int forwardOffset = 0;
//    public static Pose2d backRed = new Pose2d(58 + forwardOffset, -32 - redMidOff - shiftOffset, Math.toRadians(endAngle));
//    public static Pose2d backBlue = new Pose2d(58 + forwardOffset, 38 + blueMidOff - shiftOffset, Math.toRadians(endAngle));

    public static TrajectorySequence redShort(MecanumDrive drive) {
        int baseX = 54 + forwardOffset; //!
        int baseY = -32 - redMidOff - 1;
        switch (autoRandomReliable) {
            case mid:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)))
                        .build();
            case right:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY + rightShift, Math.toRadians(endAngle)))
                        .build();
            case left:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY + leftShift, Math.toRadians(endAngle)))
                        .build();
        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(drive.getPoseEstimate())
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive, PathLong pathLong) {
        int baseX = 58 + forwardOffset - backdropOffset;
        int baseY = -32 - 2;
        switch (pathLong) {
            case INSIDE:
                switch (autoRandomReliable) {
                    case mid:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(-52, -10, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(-52, -10, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, -12))
                                .lineTo(new Vector2d(36, -30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY + rightShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .turn(Angle.normDelta(START_POSE.getHeading() - drive.getPoseEstimate().getHeading()))
                                .lineTo(new Vector2d(drive.getPoseEstimate().getX() + 4, -10))
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, -30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY + leftShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                }
            case OUTSIDE:
                baseY += 5;
                switch (autoRandomReliable) {
                    case mid:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .splineToLinearHeading(new Pose2d(baseX, baseY + leftShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX() - xOffset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffsetRed, Math.toRadians(endAngle)))
                                .splineToLinearHeading(new Pose2d(baseX, baseY + rightShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                }
            default:
                return null;
        }
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        int baseX = 58 + forwardOffset;
        int baseY = 38 + blueMidOff;
        switch (autoRandomReliable) {
            case mid:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)))
                        .build();
            case right:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY + rightShift, Math.toRadians(endAngle)))
                        .build();
            case left:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(baseX, baseY + leftShift, Math.toRadians(endAngle)))
                        .build();
        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(drive.getPoseEstimate())
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive, PathLong pathLong) {
        int baseX = 58 + forwardOffset - backdropOffset;
        int baseY = 38 + blueMidOff;
        switch (pathLong) {
            case INSIDE:
                switch (autoRandomReliable) {
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), 12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 12))
                                .lineTo(new Vector2d(36, 30)) //!might be able to remove this
                                .splineToLinearHeading(new Pose2d(baseX, baseY + leftShift / 2, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case mid:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 10))
                                .lineTo(new Vector2d(36, 30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), 12, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 12))
                                .lineTo(new Vector2d(36, 30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY + rightShift - 2, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                }
            case OUTSIDE:
                baseY -= 4;
                switch (autoRandomReliable) {
                    case mid:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case left:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY + leftShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                    case right:
                        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(AutoServoPositions.flipUp, flipServo))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffsetBlue, Math.toRadians(endAngle)))
                                .lineTo(new Vector2d(36, 30))//!
                                .splineToLinearHeading(new Pose2d(baseX, baseY + rightShift, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                                .build();
                }
            default:
                return null;
        }
    }
}
