package org.firstinspires.ftc.teamcode.opModes.auto.cycleAutos.endIn.outsideBackPath;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.cycleMachine;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.fullAutoI_OP_Sort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous(group = fullAutoI_OP_Sort, preselectTeleOp = preselect)
//@Disabled
public class FullBLOpI extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        StateMachine<autoPatterns.cycleStates> machine = cycleMachine(drive, PathLong.OUTSIDE, EndPose.RIGHT);
        robot.initAuto(hardwareMap, this, true);
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
        }
    }
}