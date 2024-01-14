package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.raiseArm;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class generalPatterns {
    // method to go to the backdrop
    public static void navToBackdrop_Place(MecanumDrive drive, boolean raiseArm) {
        calculateFlipPose(60, flipServo);
        if (raiseArm) {
            raiseArm();
        }
        switch (StartPose.alliance) {
            case RED:
                switch (StartPose.side) {
                    case LEFT:
                        drive.followTrajectorySequence(BackdropTrajectories.redLong(drive));
                        break;
                    case RIGHT:
                        drive.followTrajectorySequence(BackdropTrajectories.redShort(drive));
                        break;
                }
                break;
            case BLUE:
                switch (StartPose.side) {
                    case LEFT:
                        drive.followTrajectorySequence(BackdropTrajectories.blueShort(drive));
                        break;
                    case RIGHT:
                        drive.followTrajectorySequence(BackdropTrajectories.blueLong(drive));
                        break;
                }
                break;
        }
        shiftAuto(drive);
        ServoUtil.calculateFlipPose(25, flipServo);
        ServoUtil.openClaw(claw1);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(8)
                .build());
    }

    // drive and place first pixel
    public static void SpikeNav(MecanumDrive drive) {
        switch (autoHardware.autonomousRandom) {
            case left:
                //LEFT RED
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(22)
                        .turn(Math.toRadians(50))
                        .addDisplacementMarker(() -> {
                            ServoUtil.openClaw(HardwareConfig.claw2);
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .back(1)
                        .build()
                );
                if (StartPose.alliance == Alliance.BLUE && StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .strafeLeft(13)
                            .build()
                    );
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.left;
                break;
            case mid:
                //left
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(26)
                        .addDisplacementMarker(() -> {
                            ServoUtil.openClaw(HardwareConfig.claw2);
                            ServoUtil.calculateFlipPose(30, flipServo);
                        })
                        .back(1)
                        .build());
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.mid;
                break;
            case right:
                // all left sides
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(22)
                                .turn(Math.toRadians(-60))
                                .addDisplacementMarker(() -> {
                                    ServoUtil.openClaw(HardwareConfig.claw2);
                                    ServoUtil.calculateFlipPose(30, flipServo);
                                })
                                .back(1)
                                .build());
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.right;
                break;
        }
    }
}
