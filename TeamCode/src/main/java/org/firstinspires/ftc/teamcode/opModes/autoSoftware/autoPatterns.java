package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Sensors.driveByPotentVal;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoBase;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getCycleSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

public class autoPatterns {
    public static void halfAutoShort(MecanumDrive drive) {
        flipServoBase(flipServo);
//        driveByPotentVal(5, HardwareConfig.potentiometer, motorRotation);
        SpikeNav(drive);
//        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
//            // long side
//            getCycleSpot();
//        }
        navToBackdrop(drive);
        shiftAuto(drive);
        ServoUtil.openClaw(claw1);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        updatePose(drive);
    }
    public static void halfAutoLong(MecanumDrive drive) {
        flipServoBase(flipServo);
        driveByPotentVal(5, HardwareConfig.potentiometer, motorRotation);
        SpikeNav(drive);
//        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
//            // long side
//            getCycleSpot();
//        }
//        navToBackdrop(drive);
//        shiftAuto(drive);
//        ServoUtil.openClaw(claw1);
//        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
//        updatePose(drive);
    }

    public static void cycle(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) {
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, PoseStorage.currentPose));
            } else {
                //long side
            }
        }
    }
}
