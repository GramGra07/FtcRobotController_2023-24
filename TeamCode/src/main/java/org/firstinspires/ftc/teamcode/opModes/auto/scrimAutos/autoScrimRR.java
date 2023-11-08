package org.firstinspires.ftc.teamcode.opModes.auto.scrimAutos;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorFlipper;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.autonomousRandom;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.driveByPotentVal;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.webcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Trajectories.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesLEFT;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.ColorEdgeDetectionBounded;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "scrim")
//@Disabled
public class autoScrimRR extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.RIGHT));
        robot.initAuto(hardwareMap,StartPose.alliance);
        if (isStopRequested()) return;
        waitForStart();
        driveByPotentVal(75, HardwareConfig.potentiometer,motorFlipper);
        SpikeNav(drive);
        navToBackdrop(drive);
//        shiftAuto(drive);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}