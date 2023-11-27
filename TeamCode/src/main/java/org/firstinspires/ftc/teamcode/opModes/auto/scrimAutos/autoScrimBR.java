package org.firstinspires.ftc.teamcode.opModes.auto.scrimAutos;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.SpikeNav;
import static org.firstinspires.ftc.teamcode.Sensors.driveByPotentVal;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.cycle;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.halfAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.navToBackdrop;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoHardware.webcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous(group = "ascrim")
//@Disabled
public class autoScrimBR extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.RIGHT));
        robot.initAuto(hardwareMap,this);
        if (opModeIsActive()) {
            halfAuto(drive);
            cycle(drive);
        }
        webcam.closeCameraDevice();
    }
}