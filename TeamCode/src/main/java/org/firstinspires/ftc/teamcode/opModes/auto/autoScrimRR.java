package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesLEFT;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesRIGHT;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous
//@Disabled
public class autoScrimRR extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.RIGHT));
        robot.initAuto(hardwareMap);
        if (isStopRequested()) return;
        waitForStart();
        switch (autoHardware.autonomousRandom) {
            case left:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    if (StartPose.alliance == Alliance.RED) {
                        drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeLeftLRed(drive));
                    } else {
                        drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeLeftLBlue(drive));
                    }
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeLeftR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case mid:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case right:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeRightL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeRightR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            default:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}