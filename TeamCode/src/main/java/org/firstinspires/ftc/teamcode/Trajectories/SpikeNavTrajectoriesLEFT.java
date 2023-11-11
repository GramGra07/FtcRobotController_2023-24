package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesLEFT {
    public static TrajectorySequence navToSpikeRightL(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(31)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(16)
                .build();
    }

    public static TrajectorySequence navToSpikeLeftLRed(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(28)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(6)
                .build();
    }

    public static TrajectorySequence navToSpikeLeftLBlue(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(6)
                .strafeLeft(18)
                .forward(15)
                .build();
    }

    public static TrajectorySequence navToSpikeCenterL(MecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(29)
                .strafeLeft(2)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(HardwareConfig.claw2);
                })
                .back(5)
                .build();
    }
}
