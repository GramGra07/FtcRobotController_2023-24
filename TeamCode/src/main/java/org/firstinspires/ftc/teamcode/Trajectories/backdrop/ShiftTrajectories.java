package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.fwd;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class ShiftTrajectories {
    public static TrajectorySequence shiftLeft(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(claw2);
                    ServoUtil.calculateFlipPose(10, flipServo);
                })
                .forward(fwd)
                .strafeLeft(8)
                .build();
    }

    public static TrajectorySequence shiftRight(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addDisplacementMarker(() -> {
                    ServoUtil.closeClaw(claw2);
                    ServoUtil.calculateFlipPose(10, flipServo);
                })
                .forward(fwd)
                .strafeRight(8)
                .build();
    }
}
