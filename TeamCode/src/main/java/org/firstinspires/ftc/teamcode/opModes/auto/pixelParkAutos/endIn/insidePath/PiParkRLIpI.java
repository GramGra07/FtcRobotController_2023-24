package org.firstinspires.ftc.teamcode.opModes.auto.pixelParkAutos.endIn.insidePath;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.pixelParkMachine;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.piParkISort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.gentrifiedApps.statemachineftc.StateMachine;

@Autonomous(group = piParkISort, preselectTeleOp = preselect)
//@Disabled
public class PiParkRLIpI extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.LEFT));
        StateMachine<autoPatterns.pixelParkStates> machine = pixelParkMachine(drive, PathLong.INSIDE, EndPose.LEFT);
        robot.initAuto(hardwareMap, this, false);
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
        }
    }
}