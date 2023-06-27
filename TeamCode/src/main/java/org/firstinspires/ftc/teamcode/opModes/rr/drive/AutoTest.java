package org.firstinspires.ftc.teamcode.opModes.rr.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // @link https://learnroadrunner.com/trajectorybuilder-functions.html#forward-distance-double
        //pause
        // @link https://learnroadrunner.com/markers.html#types-of-markers
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(startPose)
                .strafeRight(10)
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end())
                .strafeLeft(10)
                .build();

        Trajectory splineMovement = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20, 9), Math.toRadians(90))
                .splineTo(new Vector2d(20, 9), Math.toRadians(180))
                .build();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(36, 36), Math.toRadians(0))
                .build();
        // if used a drive.turn method, follow below to make it change radians
        //Trajectory traj2 = drive.trajectoryBuilder(last traj.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
        //        .strafeLeft(10)
        //        .build();
        //set it slower and limit to 15 in/s
        //.splineTo(
        //        new Vector2d(30, 30), 0,
        //        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        //        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        //)

        //drive.forward(in)
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        //drive.turn(Math.toRadians(90));
        drive.followTrajectory(myTrajectory2);
    }
}