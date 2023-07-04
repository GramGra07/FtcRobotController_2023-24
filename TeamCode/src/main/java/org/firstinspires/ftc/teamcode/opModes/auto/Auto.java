package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    @Override
    public void runOpMode() {
        // @link https://learnroadrunner.com/trajectorybuilder-functions.html#forward-distance-double
        //pause
        // @link https://learnroadrunner.com/markers.html#types-of-markers
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(60.09, -0.55), Math.toRadians(90.00))
                .splineTo(new Vector2d(59.18, 20.70), Math.toRadians(91.71))
                .splineTo(new Vector2d(30.60, 39.02), Math.toRadians(188.41))
                .splineTo(new Vector2d(-2.93, 35.18), Math.toRadians(180.00))
                .splineTo(new Vector2d(-11.91, 21.07), Math.toRadians(268.04))
                .splineTo(new Vector2d(-26.02, 11.91), Math.toRadians(183.84))
                .splineTo(new Vector2d(-34.44, -2.56), Math.toRadians(-85.50))
                .splineTo(new Vector2d(-24.92, -11.36), Math.toRadians(-4.89))
                .splineTo(new Vector2d(-11.91, -61.37), Math.toRadians(-87.17))
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

        drive.followTrajectorySequence(traj1);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}