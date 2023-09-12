package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.autonomousRandom;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.delayUntilTagFound;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.randTag;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.runSpikeNav;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.Vision;
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
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap);
        if (isStopRequested()) return;
        runSpikeNav(drive, this);
        // do claw stuff to move & drop pixel
        navToBackdrop(drive);
        // drop pixel
        delayUntilTagFound(this,randTag);
        // do we need new pose?
        // move claw up
        switch (autonomousRandom) {
            case left:
                ShiftTrajectories.shiftLeft(drive);
                break;
            case mid:
                // should already be lined up
                break;
            case right:
                ShiftTrajectories.shiftRight(drive);
                break;
        }
        // drop pixel
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}