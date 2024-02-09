package org.firstinspires.ftc.teamcode.Trajectories;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class spikeNavTraj {
    public static TrajectorySequence midPiNav(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(25)
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    ServoUtil.calculateFlipPose(30, flipServo);
                })
                .back(4)
                .build();
    }

    public static TrajectorySequence fwdTLeft(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(22)
                .turn(Math.toRadians(50))
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    ServoUtil.calculateFlipPose(30, flipServo);
                })
                .back(1)
                .turn(Math.toRadians(-50))
                .build();
    }

    public static int rightAngle = 55;

    public static TrajectorySequence fwdTRight(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(22)
                .turn(Math.toRadians(-rightAngle))
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    ServoUtil.calculateFlipPose(30, flipServo);
                })
                .back(1)
                .turn(Math.toRadians(rightAngle))
                .build();
    }
}
