package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

public class endPose {

    // contains positions for the robot to go to at the end of the auto
    public static Pose2d endPoseRightRed = new Pose2d(50,-58,Math.toRadians(0));
    public static Pose2d endPoseLeftRed = new Pose2d(50,-12,Math.toRadians(0));
    public static Pose2d endPoseRightBlue = new Pose2d(45,12,Math.toRadians(0));
    public static Pose2d endPoseLeftBlue = new Pose2d(54,48,Math.toRadians(0));

    // returns a trajectory sequence to go to the end pose
    public static TrajectorySequence goToEndPose(Pose2d pose, MecanumDrive drive){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
    }
}
