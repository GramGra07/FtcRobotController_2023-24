package org.firstinspires.ftc.teamcode.opModes.auto.constant.parkAuto;

import static org.firstinspires.ftc.teamcode.opModes.auto.constant.parkAuto.parkUtil.parkUtil.parkAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.parkSort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = parkSort, preselectTeleOp = preselect)
@Disabled
public class parkAutoBR extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.RIGHT));
        robot.initAuto(hardwareMap, this, false);
        if (opModeIsActive()) {
            parkAuto(drive);
        }
        goToEndPose(EndPose.NONE, drive);
    }
}