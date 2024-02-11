package org.firstinspires.ftc.teamcode.Trajectories.backdrop;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.fwd;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class ShiftTrajectories {
    public static int leftOffset = 1;
    public static int strafe = 12;
    public static int shiftOffset = 0;
    public static int leftShift = 8;
    public static int rightShift = 8;

    public static TrajectorySequence shiftForward(MecanumDrive drive) {
        updatePose(drive);
        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .forward(8)
                .build();
    }

    public static void getShift() {
        if (autoRandomReliable == AutoRandom.left) {
            shiftOffset = -leftShift;
        } else if (autoRandomReliable == AutoRandom.right) {
            shiftOffset = rightShift;
        } else {
            shiftOffset = 0;
        }
    }

//    public static TrajectorySequence shiftLeft(MecanumDrive drive) {
//        updatePose(drive);
//        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                .addDisplacementMarker(() -> {
//                    ServoUtil.closeClaw(claw2);
//                    if (StartPose.alliance == Alliance.BLUE && StartPose.side == StartSide.LEFT) {
//                        strafe = 1;
//                    }
//                })
//                .forward(fwd - leftOffset)
//                .strafeLeft(strafe)
//                .build();
//    }
//
//    public static TrajectorySequence shiftRight(MecanumDrive drive) {
//        updatePose(drive);
//        return drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                .addDisplacementMarker(() -> {
//                    ServoUtil.closeClaw(claw2);
//                })
//                .forward(fwd)
//                .strafeRight(12)
//                .build();
//    }
}
