package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.useAutoClose;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime.autoCloseDelay;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.lastTimeOpen;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.slowModeIsOn;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.updateStatus;

import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class DriverAid {
    public static void operateClawByDist(boolean inAuto) {
        double val = ServoUtil.autoCloseDist;
        if (!inAuto) {
            if (useAutoClose) {
                if (lastTimeOpen + autoCloseDelay < HardwareConfig.timer.seconds()) {
                    if (HardwareConfig.distance1 < val && !HardwareConfig.claw1Possessed) {
                        closeClaw(HardwareConfig.claw1);
                    } else if (HardwareConfig.distance2 < val && !HardwareConfig.claw2Possessed) {
                        closeClaw(HardwareConfig.claw2);
                    }
                }
            }
        } else {
            if (HardwareConfig.distance1 < 5) {
                closeClaw(HardwareConfig.claw1);
            } else if (HardwareConfig.distance2 < 5) {
                closeClaw(HardwareConfig.claw2);
            }
        }
    }

    public static void doDriverAid(MecanumDrive drive, boolean goToDrone, boolean turnStraight, boolean turnWing, boolean breakFollowing) {
        if (goToDrone) {
            slowModeIsOn = false;
            IsBusy.isAutoInTeleop = true;
            updateStatus("Auto-ing");
            drive.update();
            // go to drone scoring position]
        }
        if (turnStraight) {
            slowModeIsOn = false;
            IsBusy.isAutoInTeleop = true;
            updateStatus("Auto-ing");
            drive.turnAsync(Angle.normDelta(Math.toRadians(0) - drive.getPoseEstimate().getHeading()));
        }
        if (turnWing) {
            slowModeIsOn = false;
            IsBusy.isAutoInTeleop = true;
            updateStatus("Auto-ing");
            if (StartPose.alliance == Alliance.RED) {
                drive.turnAsync(Angle.normDelta(Math.toRadians(135) - drive.getPoseEstimate().getHeading()));
            } else {
                drive.turnAsync(Angle.normDelta(Math.toRadians(-135) - drive.getPoseEstimate().getHeading()));
            }
        }
        if (!drive.isBusy()) {
            IsBusy.isAutoInTeleop = false;
            updateStatus("Running");
        }
        if (breakFollowing) {
            IsBusy.isAutoInTeleop = false;
            updateStatus("Running");
            drive.breakFollowing();
        }
    }
//    int poseX = 50, poseY = 50, turn = 135;
//                int quadrant = getQuadrant(drive.getPoseEstimate());
//                Pose2d redWing = new Pose2d(-poseX, poseY, Math.toRadians(turn));
//                Pose2d blueWing = new Pose2d(-poseX, -poseY, Math.toRadians(-turn));
//                Pose2d blueRiggingInside = new Pose2d(-12, -36, Math.toRadians(0));
//                Pose2d redRiggingInside = new Pose2d(-12, 36, Math.toRadians(0));
//                Pose2d zeroZero = new Pose2d(0, 0, Math.toRadians(0));
//                if (StartPose.alliance == Alliance.RED) {
//                    if (quadrant == 1) {
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .splineToLinearHeading(zeroZero, Math.toRadians(180))
//                                .forward(30)
//                                .splineToLinearHeading(redWing, redWing.getHeading())
//                                .build());
//                    }
//                    if (quadrant == 2) {
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .splineToSplineHeading(blueRiggingInside, Math.toRadians(180))
//                                .forward(30)
//                                .splineToLinearHeading(redWing, redWing.getHeading())
//                                .build());
//                    }
//                    if ((quadrant == 3) || (quadrant == 4)) {
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .splineToLinearHeading(redWing, redWing.getHeading())
//                                .build());
//                    }
//                } else {
//                    if (quadrant == 1) {
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .splineToLinearHeading(blueRiggingInside, Math.toRadians(180))
//                                .forward(30)
//                                .splineToLinearHeading(blueWing, blueWing.getHeading())
//                                .build());
//                    }
//                    if (quadrant == 2) {
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .splineToSplineHeading(zeroZero, Math.toRadians(180))
//                                .forward(30)
//                                .splineToLinearHeading(blueWing, blueWing.getHeading())
//                                .build());
//                    }
//                    if ((quadrant == 4) || (quadrant == 3)) {
//                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .splineToLinearHeading(blueWing, blueWing.getHeading())
//                                .build());
//                    }
//                }
//            }
}
