package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.UtilClass.StateMachineTest.state.STATE_two;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;

import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

import StateMachine.StateMachine;

public class exampleSm {

    public static StateMachine<StateMachineTest.state> machine(MecanumDrive drive) {
        StateMachine.Builder<StateMachineTest.state> builder = new StateMachine.Builder<>();
        return builder
                .state(StateMachineTest.state.STATE_one)
                .onEnter(StateMachineTest.state.STATE_one, () -> {
                    ServoUtil.calculateFlipPose(0, flipServo);
                    SpikeNav(drive, PathLong.NONE);
                    drive.update();
                })
                .whileState(StateMachineTest.state.STATE_one, () -> !drive.isBusy())
                .transition(StateMachineTest.state.STATE_one, () -> !drive.isBusy())
                .state(STATE_two)
                .onEnter(STATE_two, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                    goToEndPose(EndPose.StartingPosition, drive);
                    drive.update();
                })
                .whileState(STATE_two, () -> !drive.isBusy())
                .transition(STATE_two, () -> !drive.isBusy())
                .stopRunning()
                .build();
    }
}