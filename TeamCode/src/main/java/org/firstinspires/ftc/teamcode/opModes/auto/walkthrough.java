package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.trajectorysequence.TrajectorySequence;

@Autonomous
//@Disabled
public class walkthrough extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);
    int randTag;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        //create trajectories here
        TrajectorySequence navToSpikeLRed = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        TrajectorySequence navToSpikeLBlue = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        TrajectorySequence navToSpikeMRed = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        TrajectorySequence navToSpikeMBlue = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        TrajectorySequence navToSpikeRRed = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        TrajectorySequence navToSpikeRBlue = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13.01, -25.47), Math.toRadians(90.00))
                .build();
        robot.initAuto(hardwareMap, true);
        if(isStopRequested()) return;
        while (opModeIsActive()) {
            // look for april tag with dif id
            Vision.searchAprilTags(Vision.ourTag);
            // find it take which side, left, right, center
            Vision.getPoseFromCenter(Vision.ourTag);

            switch (autoHardware.autonomousRandom) {
                case left:
                    // move to left side
                    Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
                    Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,false);
                    Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,false);

                    break;
                case mid:
                    // move to mid side
                    Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
                    Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,true);
                    Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,false);
                    break;
                case right:
                    // move to right side
                    Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
                    Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,true);
                    Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,true);
                    break;
                default:
                    Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,false);
                    Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,false);
                    Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,false);
            }
            Vision.telemetryAprilTag(this);
        }
        // move and place to that side
        // move all the way to backdrop
        switch (autoHardware.autonomousRandom) {
            case left:
                // move less

                break;
            case mid:
                // move to mid
                break;
            case right:
                // move all the way right
                break;
        }
        // drop pixel

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}