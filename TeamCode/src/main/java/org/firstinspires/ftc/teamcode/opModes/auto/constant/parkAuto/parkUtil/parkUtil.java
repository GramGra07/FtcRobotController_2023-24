package org.firstinspires.ftc.teamcode.opModes.auto.constant.parkAuto.parkUtil;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class parkUtil {
    public static void parkAuto(MecanumDrive drive) {
        if (StartPose.alliance == Alliance.BLUE) {
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .strafeLeft(48)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.BLUE) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeLeft(84)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeRight(84)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(4)
                                .strafeRight(48)
                                .build()
                );
            }
        }
    }
}
