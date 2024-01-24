package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.START_POSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class endPose {

    // contains positions for the robot to go to at the end of the auto
    public static int outside = 64;
    public static int inside = 12;
    public static Pose2d endPoseRightRed = new Pose2d(50, -outside, Math.toRadians(0));
    public static Pose2d endPoseLeftRed = new Pose2d(50, -inside, Math.toRadians(0));
    public static Pose2d endPoseRightBlue = new Pose2d(50, inside, Math.toRadians(0));
    public static Pose2d endPoseLeftBlue = new Pose2d(50, outside, Math.toRadians(0));

    // returns a trajectory sequence to go to the end pose
    public static void goToEndPose(EndPose endPose, MecanumDrive drive) {
        Pose2d pose = drive.getPoseEstimate();
        switch (endPose) {
            case StartingPosition:
                pose = START_POSE;
                break;
            case LEFT:
                switch (StartPose.alliance) {
                    case RED:
                        pose = endPoseLeftRed;
                        break;
                    case BLUE:
                        pose = endPoseLeftBlue;
                        break;
                }
                break;
            case RIGHT:
                switch (StartPose.alliance) {
                    case RED:
                        pose = endPoseRightRed;
                        break;
                    case BLUE:
                        pose = endPoseRightBlue;
                        break;
                }
                break;
        }
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build()
        );
    }
}
