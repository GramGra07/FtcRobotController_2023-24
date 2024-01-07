package org.firstinspires.ftc.teamcode.opModes.auto.pixelParkAutos;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.visionPortal;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.webcam;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.pixelPark;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = "ascrim", preselectTeleOp="teleOp")
//@Disabled
public class autoPiParkBL extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap,this);
        if (opModeIsActive()) {
            pixelPark(drive);
        }
        updatePose(drive);
        webcam.closeCameraDevice();
    }
}