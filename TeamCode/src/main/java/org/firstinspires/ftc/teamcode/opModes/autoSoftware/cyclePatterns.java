package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.endAngle;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.offset;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetBlue;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.startOffsetRed;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
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
        int clawAngle = 15;
        switch (pathLong) {
            case INSIDE:
                switch (StartPose.alliance) { // switch statement to determine the alliance
                    case RED:
                        spot = new Pose2d(-54.5, -10, Math.toRadians(180));
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(36, -12, spot.getHeading()))
                                .lineToLinearHeading(spot)
                                .addSpatialMarker(new Vector2d(-36, -10), () ->
                                        calculateFlipPose(clawAngle, flipServo))
                                .addDisplacementMarker(() -> {
                                    closeClaw(claw1);
                                    closeClaw(claw2);
                                })
                                .back(1)
                                .build()
                        );
                        break;
                    case BLUE:
                        spot = new Pose2d(-54.5, 10.1, Math.toRadians(180));
                        drive.followTrajectorySequenceAsync((drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(36, 12, spot.getHeading()))
                                .lineToLinearHeading(spot)
                                .addSpatialMarker(new Vector2d(-36, 10), () ->
                                        calculateFlipPose(clawAngle, flipServo))
                                .addDisplacementMarker(() -> {
                                    closeClaw(claw1);
                                    closeClaw(claw2);
                                })
                                .back(1)
                                .build()
                        ));
                        break;
                }
                break;
            case OUTSIDE:
                switch (StartPose.alliance) {
                    case RED:
                        spot = new Pose2d(-57, -31, Math.toRadians(180));
                        drive.followTrajectorySequenceAsync((drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() + startOffsetRed, spot.getHeading()))
                                .splineToLinearHeading(new Pose2d(-36, START_POSE.getY() + startOffsetRed, spot.getHeading()), spot.getHeading())
                                .splineToLinearHeading(spot, spot.getHeading())
                                .addSpatialMarker(new Vector2d(-36, -58), () ->
                                        calculateFlipPose(clawAngle, flipServo))
                                .addDisplacementMarker(() -> {
                                    closeClaw(claw1);
                                    closeClaw(claw2);
                                })
                                .back(1)
                                .build()
                        ));
                        break;
                    case BLUE:
                        spot = new Pose2d(-55, 31, Math.toRadians(180));
                        drive.followTrajectorySequenceAsync((drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(START_POSE.getX(), START_POSE.getY() - startOffsetBlue, spot.getHeading()))
                                .splineToLinearHeading(new Pose2d(-36, START_POSE.getY() - startOffsetBlue - 1, spot.getHeading()), spot.getHeading())
                                .splineToLinearHeading(spot, spot.getHeading())
                                .addSpatialMarker(new Vector2d(-36, 58), () ->
                                        calculateFlipPose(clawAngle, flipServo))
                                .addDisplacementMarker(() -> {
                                    closeClaw(claw1);
                                    closeClaw(claw2);
                                })
                                .back(1)
                                .build()
                        ));
                        break;
                }
                break;
        }

    }
}
