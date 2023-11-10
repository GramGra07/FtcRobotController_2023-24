package org.firstinspires.ftc.teamcode.Trajectories;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.blueRotate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Config
public class BackdropTrajectories {
    public static int endAngle = 0;
    public static double displacementBlueLong = 20;
    public static double displacementRedShort = 20;
    public static double displacementRedLong = 20;
    public static double displacementBlueShort = 20;
    public static int potentBackTarget = 270;

    public static TrajectorySequence redShort(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(displacementRedShort, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(endAngle)))
                .build();
    }

    public static TrajectorySequence redLong(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(displacementRedLong, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(endAngle)))
                .build();
    }

    public static TrajectorySequence blueShort(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(displacementBlueShort, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(endAngle)))
                .build();
    }

    public static TrajectorySequence blueLong(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(blueRotate+90)))
                .forward(56)
                .addDisplacementMarker(displacementBlueLong, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    // bring arm up
                })
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(endAngle)))
//                .addDisplacementMarker(()->{
//                    autoHardware.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorFlipper);
//                    ServoUtil.closeClaw(HardwareConfig.claw1);
//                })
                .build();
    }
}
