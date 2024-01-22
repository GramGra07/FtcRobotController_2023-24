package org.firstinspires.ftc.teamcode.opModes.auto.pixelParkAutos.endIn.shortSide;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.endAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.pixelPark;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.piParkShort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = piParkShort, preselectTeleOp = preselect)
//@Disabled
public class PiParkBLI extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap, this);
        if (opModeIsActive()) {
            pixelPark(drive, PathLong.NONE);
        }
        endAuto(EndPose.RIGHT, drive);
    }
}