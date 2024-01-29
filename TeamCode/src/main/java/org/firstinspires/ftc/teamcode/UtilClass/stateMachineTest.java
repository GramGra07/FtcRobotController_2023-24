package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

import StateMachineFTC.StateMachine;


@Autonomous(group = place1Sort, preselectTeleOp = preselect)
//@Disabled
public class stateMachineTest extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    public enum state {
        INIT,
        SPIKE_NAV,
        END_POSE,
        STOP,
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap, this, false);
        StateMachine<state> machine = new StateMachine.Builder<state>()
                .state(state.SPIKE_NAV)
                .onEnter(state.SPIKE_NAV, () -> {
                    ServoUtil.calculateFlipPose(0, flipServo);
                })
                .whileState(state.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    SpikeNav(drive, PathLong.NONE);
                    drive.update();
                })
                .transition(state.SPIKE_NAV, () -> !drive.isBusy())
                .state(state.END_POSE)
                .onEnter(state.END_POSE, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                })
                .whileState(state.END_POSE, () -> !drive.isBusy(), () -> {
                    goToEndPose(EndPose.StartingPosition, drive);
                    drive.update();
                })
                .transition(state.END_POSE, () -> !drive.isBusy())
                .state(state.STOP)
                .stopRunning(state.STOP)
                .build();
        waitForStart();
        machine.start();
        while (machine.mainLoop(this)) {
            machine.update();
            drive.update();
        }
    }
}