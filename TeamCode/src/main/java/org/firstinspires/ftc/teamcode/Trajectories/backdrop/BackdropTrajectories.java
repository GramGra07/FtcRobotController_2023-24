package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autonomousRandom;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.raiseArm;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config
public class BackdropTrajectories {
    public static int endAngle = 0;

    public static TrajectorySequence redShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .lineToLinearHeading(new Pose2d(56, -30, Math.toRadians(endAngle)))
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, -12))
                .lineTo(new Vector2d(36, -12))
                .addDisplacementMarker(() -> raiseArm())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .splineToLinearHeading(new Pose2d(56, -36, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                .build();
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .lineToLinearHeading(new Pose2d(54, 32, Math.toRadians(endAngle)))
//                .addDisplacementMarker(()->{
//                    if (autoRandomReliable == AutoRandom.left){
//                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeRight(9).forward(4).build());
//                    }
//                })
//                .back(1)
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, 12))
                .lineTo(new Vector2d(36, 12))
                .addDisplacementMarker(() -> raiseArm())
                .addDisplacementMarker(() -> ServoUtil.calculateFlipPose(30, flipServo))
                .splineToLinearHeading(new Pose2d(54, 36, Math.toRadians(endAngle)), Math.toRadians(endAngle))
                .build();
    }
}
