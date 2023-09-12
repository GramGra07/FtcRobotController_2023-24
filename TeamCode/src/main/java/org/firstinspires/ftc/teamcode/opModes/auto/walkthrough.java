package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectories;
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
    int randTag;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        //create trajectories here

        robot.initAuto(hardwareMap, true);
        if (isStopRequested()) return;
        while (!Vision.searchAprilTags(Vision.ourTag)) {
            Vision.searchAprilTags(Vision.ourTag);
            Vision.telemetryOneTag(this, Vision.ourTag);
        }
        // look for april tag with dif id
        Vision.searchAprilTags(Vision.ourTag);
        // find it take which side, left, right, center
        Vision.getPoseFromCenter(Vision.ourTag);
        switch (autoHardware.autonomousRandom) {
            case left:
                // move to left side
                if (StartPose.alliance == Alliance.RED) {
                    randTag = Vision.leftR;
                } else {
                    randTag = Vision.leftB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                SpikeNavTrajectories.navToSpikeLeft(drive).start();
                break;
            case mid:
                // move to mid side
                if (StartPose.alliance == Alliance.RED) {
                    randTag = Vision.midR;
                } else {
                    randTag = Vision.midB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                SpikeNavTrajectories.navToSpikeCenter(drive).start();
                break;
            case right:
                // move to right side
                if (StartPose.alliance == Alliance.RED) {
                    randTag = Vision.rightR;
                } else {
                    randTag = Vision.rightB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                SpikeNavTrajectories.navToSpikeRight(drive).start();
                break;
            default:
                if (StartPose.alliance == Alliance.RED) {
                    randTag = Vision.midR;
                } else {
                    randTag = Vision.midB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                SpikeNavTrajectories.navToSpikeCenter(drive).start();
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