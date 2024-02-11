package org.firstinspires.ftc.teamcode.opModes.auto.cycleAutos.endIn.insideBackPath;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.cycleMachine;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.fullAutoI_IP_Sort;
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

@Autonomous(group = fullAutoI_IP_Sort, preselectTeleOp = preselect)
@Disabled
public class FullRRIpI extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose; // get the starting pose
    autoHardware robot = new autoHardware(this); // initialize the robot class

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.RIGHT)); // set the starting pose
        StateMachine<autoPatterns.cycleStates> machine = cycleMachine(drive, PathLong.INSIDE, EndPose.LEFT);
        robot.initAuto(hardwareMap, this, true);
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
        }
    }
}