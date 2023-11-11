package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesRIGHT {
    public static TrajectorySequence navToSpikeRightR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(12)
                .forward(24)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(16)
                .build();
    }

    public static TrajectorySequence navToSpikeLeftR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(31)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(6)
                .build();
    }

    public static TrajectorySequence navToSpikeCenterR(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(28)
                .strafeLeft(2)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(5)
                .build();
    }
}
