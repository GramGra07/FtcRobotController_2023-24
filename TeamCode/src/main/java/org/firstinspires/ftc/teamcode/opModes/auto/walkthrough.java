package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous
//@Disabled
public class walkthrough extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        //create trajectories here

        robot.initAuto(hardwareMap, true);
        if(isStopRequested()) return;
        // look for april tag with dif id
        Vision.searchAprilTags(Vision.ourTag);
        // find it take which side, left, right, center
        Vision.getPoseFromCenter(Vision.ourTag);
        switch (autoHardware.autonomousRandom) {
            case left:
                // move to left side
                HardwareConfig.green1.setState(true);
                HardwareConfig.red1.setState(false);
                break;
            case mid:
                // move to mid side
                HardwareConfig.green1.setState(true);
                HardwareConfig.red1.setState(false);
                HardwareConfig.green2.setState(true);
                HardwareConfig.red2.setState(false);
                break;
            case right:
                // move to right side
                HardwareConfig.green1.setState(true);
                HardwareConfig.red1.setState(false);
                HardwareConfig.green2.setState(true);
                HardwareConfig.red2.setState(false);
                HardwareConfig.green3.setState(true);
                HardwareConfig.red3.setState(false);
                break;
        }
        stop();
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