package org.firstinspires.ftc.teamcode.opModes.camera;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.aprilTagProcessor;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.doAprilTagPoseCorrection;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@TeleOp(group = "aa")
//@Disabled
public class aprilTest extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.initAuto(hardwareMap, this, true);
        while (opModeIsActive()) {
            Vision.telemetryAprilTag(this, aprilTagProcessor);
            doAprilTagPoseCorrection(aprilTagProcessor, telemetry, drive);
            telemetry.update();
        }
    }
}
