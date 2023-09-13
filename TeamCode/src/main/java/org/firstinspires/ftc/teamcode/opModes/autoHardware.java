package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesLEFT;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesRIGHT;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

public class autoHardware extends HardwareConfig {//auto version of hardware config
    public static double robotWidth = 16;
    public static double robotLength = 18;

    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
//  public static Pose2d startPose = getStartPose(StartPose.redRight);
    public static int targetTag = 0;
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }
    public static AutoRandom autonomousRandom = AutoRandom.mid;

    public void initAuto(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        init(ahwMap);
        Vision.initVision(ahwMap);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        myOpMode.waitForStart();
        timer.reset();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColor()));
    }
    public static Pose2d getStartPose(Alliance alliance, StartSide side){
        StartPose.alliance = alliance;
        StartPose.side = side;
        switch (alliance){
            case RED:
                switch (side){
                    case LEFT:
                        return new Pose2d(-36, -72+(robotLength/2), Math.toRadians(90));
                    case RIGHT:
                        return new Pose2d(12, -72+(robotLength/2), Math.toRadians(90));
                }
            case BLUE:
                switch (side){
                    case LEFT:
                        return new Pose2d(12, 72-(robotLength/2), Math.toRadians(-90));
                    case RIGHT:
                        return new Pose2d(-36, 72-(robotLength/2), Math.toRadians(-90));
                }
        }
        return new Pose2d(0,0,0);
    }
    public static void runSpikeNav(SampleMecanumDrive drive, OpMode opMode){
        delayUntilTagFound(opMode, Vision.ourTag);
        // look for april tag with dif id
        Vision.searchAprilTags(Vision.ourTag);
        // find it take which side, left, right, center
        Vision.getPoseFromCenter(Vision.ourTag);
        switch (autoHardware.autonomousRandom) {
            case left:
                // move to left side
                if (StartPose.alliance == Alliance.RED) {
                    targetTag = Vision.leftR;
                } else {
                    targetTag = Vision.leftB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    if (StartPose.alliance == Alliance.RED) {
                        SpikeNavTrajectoriesLEFT.navToSpikeLeftLRed(drive).start();
                    } else {
                        SpikeNavTrajectoriesLEFT.navToSpikeLeftLBlue(drive).start();
                    }
                } else {
                    SpikeNavTrajectoriesRIGHT.navToSpikeLeftR(drive).start();
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case mid:
                // move to mid side
                if (StartPose.alliance == Alliance.RED) {
                    targetTag = Vision.midR;
                } else {
                    targetTag = Vision.midB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive).start();
                } else {
                    SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive).start();
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case right:
                // move to right side
                if (StartPose.alliance == Alliance.RED) {
                    targetTag = Vision.rightR;
                } else {
                    targetTag = Vision.rightB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                if (StartPose.side == StartSide.LEFT) {
                    SpikeNavTrajectoriesLEFT.navToSpikeRightL(drive).start();
                } else {
                    SpikeNavTrajectoriesRIGHT.navToSpikeRightR(drive).start();
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            default:
                if (StartPose.alliance == Alliance.RED) {
                    targetTag = Vision.midR;
                } else {
                    targetTag = Vision.midB;
                }
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive).start();
                } else {
                    SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive).start();
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
    public static void navToBackdrop(SampleMecanumDrive drive){
        // move all the way to backdrop
        switch (StartPose.alliance){
            case RED:
                switch (StartPose.side){
                    case LEFT:
                        BackdropTrajectories.redShort(drive).start();
                        break;
                    case RIGHT:
                        BackdropTrajectories.redLong(drive).start();
                        break;
                }
                break;
            case BLUE:
                switch (StartPose.side){
                    case LEFT:
                        BackdropTrajectories.blueShort(drive).start();
                        break;
                    case RIGHT:
                        BackdropTrajectories.blueLong(drive).start();
                        break;
                }
                break;
        }
    }
    public static void delayUntilTagFound(OpMode myOpMode, int tag){
        while (!Vision.searchAprilTags(tag)) {
            Vision.searchAprilTags(tag);
            Vision.telemetryOneTag(myOpMode, tag);
        }
    }
}
