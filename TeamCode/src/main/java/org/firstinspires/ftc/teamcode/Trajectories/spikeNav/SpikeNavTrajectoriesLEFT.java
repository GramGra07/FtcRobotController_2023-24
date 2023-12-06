package org.firstinspires.ftc.teamcode.Trajectories.spikeNav;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class SpikeNavTrajectoriesLEFT {
    public static TrajectorySequence navToSpikeRightL(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(22)
                .turn(Math.toRadians(-45))
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .build();
    }

    public static TrajectorySequence navToSpikeLeftLRed(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(22)
                .turn(Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .build();
    }

    public static TrajectorySequence navToSpikeLeftLBlue(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(22)
                .turn(Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .strafeLeft(18)
                .forward(15)
                .build();
    }

    public static TrajectorySequence navToSpikeCenterL(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(25)
                .addDisplacementMarker(() -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                })
                .build();
    }
}
