package org.firstinspires.ftc.teamcode.opModes.camera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = "aa")
@Disabled
public class aprilTest extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.initAuto(hardwareMap, this);
//        webcam.closeCameraDevice();
//        visionPortal.resumeStreaming();
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        while (opModeIsActive()) {
            Vision.telemetryAprilTag(this);
            telemetry.update();
        }
    }
}
