package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.endAngle;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.offset;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffset;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.spot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class cyclePatterns {

    // method to pick up from the stack of pixels
    public static void pickFromSpot(MecanumDrive drive, PathLong pathLong) {
        switch (pathLong) {
            case INSIDE:
                switch (StartPose.alliance) {
                    case RED:
                        spot = new Pose2d(-60, -10, Math.toRadians(180));
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(36, -12))
                                .lineToLinearHeading(spot)
                                .addDisplacementMarker(() -> calculateFlipPose(45, flipServo))
                                .addDisplacementMarker(() -> closeClaw(claw2))
                                .back(1)
                                .build()
                        );
                        break;
                    case BLUE:
                        spot = new Pose2d(-60, 10, Math.toRadians(180));
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(36, 12))
                                .lineToLinearHeading(spot)
                                .addDisplacementMarker(() -> calculateFlipPose(45, flipServo))
                                .addDisplacementMarker(() -> closeClaw(claw2))
                                .back(1)
                                .build()
                        );
                        break;
                }
                break;
            case OUTSIDE:
                switch (StartPose.alliance) {
                    case RED:
                        spot = new Pose2d(-60, -36, Math.toRadians(180));
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() + startOffset, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() + startOffset, Math.toRadians(endAngle)))
                                .lineToLinearHeading(spot)
                                .addDisplacementMarker(() -> calculateFlipPose(45, flipServo))
                                .addDisplacementMarker(() -> closeClaw(claw2))
                                .back(1)
                                .build()
                        );
                        break;
                    case BLUE:
                        spot = new Pose2d(-60, 36, Math.toRadians(180));
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-START_POSE.getX() - offset, START_POSE.getY() - startOffset, Math.toRadians(endAngle)))
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffset, Math.toRadians(endAngle)))
                                .lineToLinearHeading(spot)
                                .addDisplacementMarker(() -> calculateFlipPose(45, flipServo))
                                .addDisplacementMarker(() -> closeClaw(claw2))
                                .back(1)
                                .build()
                        );
                        break;
                }
                break;
        }

    }
}
