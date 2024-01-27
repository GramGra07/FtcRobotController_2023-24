package org.firstinspires.ftc.teamcode.UtilClass.state_machine;

import static org.firstinspires.ftc.teamcode.UtilClass.state_machine.StateMachineTest.state.END_POSE;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;

import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class exampleSm {

    public static StateMachine<StateMachineTest.state> machine(MecanumDrive drive) {
        StateMachine.Builder<StateMachineTest.state> builder = new StateMachine.Builder<>();
        return builder
                .state(StateMachineTest.state.SPIKE_NAV)
                .onEnter(StateMachineTest.state.SPIKE_NAV, () -> {
                    ServoUtil.calculateFlipPose(0, flipServo);
                    SpikeNav(drive, PathLong.NONE);
                    drive.update();
                })
                .whileState(StateMachineTest.state.SPIKE_NAV, () -> !drive.isBusy())
                .transition(StateMachineTest.state.SPIKE_NAV, () -> !drive.isBusy())
                .state(END_POSE)
                .onEnter(END_POSE, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                    goToEndPose(EndPose.StartingPosition, drive);
                    drive.update();
                })
                .whileState(END_POSE, () -> !drive.isBusy())
                .transition(END_POSE, () -> !drive.isBusy())
                .stopRunning()
                .build();
    }
}