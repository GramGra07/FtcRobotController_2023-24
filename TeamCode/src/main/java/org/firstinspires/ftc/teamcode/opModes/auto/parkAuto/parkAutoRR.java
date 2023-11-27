package org.firstinspires.ftc.teamcode.opModes.auto.parkAuto;

import static org.firstinspires.ftc.teamcode.opModes.auto.parkUtil.parkUtil.parkAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.webcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = "park")
//@Disabled
public class parkAutoRR extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.RIGHT));
        robot.initAuto(hardwareMap, this);
        if (opModeIsActive()) {
            parkAuto(drive);
            updatePose(drive);
        }
        webcam.closeCameraDevice();
    }
}