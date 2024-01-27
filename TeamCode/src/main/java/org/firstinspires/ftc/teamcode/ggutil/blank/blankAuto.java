package org.firstinspires.ftc.teamcode.ggutil.blank;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous
@Disabled
public class blankAuto extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.initAuto(hardwareMap, this, false);
        if (opModeIsActive()) {
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        goToEndPose(EndPose.NONE, drive);
    }
}