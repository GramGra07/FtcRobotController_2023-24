package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.potentiometer;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getCycleSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.spot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class autoPatterns {
    public static void halfAuto(MecanumDrive drive) {
        ServoUtil.calculateFlipPose(60, flipServo);
        SpikeNav(drive);
        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
            // long side
            getCycleSpot();
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(spot)
                    .addDisplacementMarker(() -> Sensors.driveByPotentVal(6, potentiometer, motorRotation))
                    .addDisplacementMarker(() -> calculateFlipPose(0, flipServo))
                    .back(1)
                    .build()
            );
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(4)
                    .build()
            );
            closeClaw(claw2);
            updatePose(drive);
        }
        navToBackdrop(drive);
        shiftAuto(drive);
        ServoUtil.openClaw(claw1);
        ServoUtil.openClaw(claw2);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        updatePose(drive);
    }

    public static void cycle(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) {
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, drive.getPoseEstimate()));
            } else {
                //long side
            }
        }
    }
}
