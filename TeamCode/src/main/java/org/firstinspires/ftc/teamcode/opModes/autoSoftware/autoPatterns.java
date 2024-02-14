package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Limits.autoExtension;
import static org.firstinspires.ftc.teamcode.Limits.autoRotation;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.blueMidOff;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.forwardOffset;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorExtension;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.startDist;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.encoderDrive;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.cyclePatterns.pickFromSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartDist;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.gentrifiedApps.statemachineftc.StateMachine;

public class autoPatterns {
    // contains different auto patterns for different tasks
    public enum place1States {
        INIT,
        SPIKE_NAV,
        END_POSE,
        STOP,
    }

    public static StateMachine<place1States> place1Machine(MecanumDrive drive) {
        StateMachine.Builder<place1States> builder = new StateMachine.Builder<>();
        return builder
                .state(place1States.SPIKE_NAV)
                .onEnter(place1States.SPIKE_NAV, () -> {
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, PathLong.NONE);
                })
                .whileState(place1States.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(place1States.SPIKE_NAV, () -> {
                    openClaw(claw2);
                })
                .transition(place1States.SPIKE_NAV, () -> !drive.isBusy(), 0)
                .state(place1States.END_POSE)
                .onEnter(place1States.END_POSE, () -> {
                    calculateFlipPose(30, flipServo);
                    goToEndPose(EndPose.StartingPosition, drive);
                })
                .whileState(place1States.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(place1States.END_POSE, () -> !drive.isBusy(), 0)
                .stopRunning(place1States.STOP)
                .build();
    }

    // does two pixel and then goes to the end pose
    public enum pixelParkStates {
        INIT,
        SPIKE_NAV,
        BACKDROP,
        SHIFT,
        END_POSE,
        RETRACT,
        STOP,
    }

    public static int rotate = 0;
    public static int extend = 0;

    public static StateMachine<pixelParkStates> pixelParkMachine(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        StateMachine.Builder<pixelParkStates> builder = new StateMachine.Builder<>();
        return builder
                .state(pixelParkStates.INIT)
                .onEnter(pixelParkStates.INIT, () -> {
                })
                .transition(pixelParkStates.INIT, () -> true, 0)// if we want to make it delay before entering
                .state(pixelParkStates.SPIKE_NAV)
                .onEnter(pixelParkStates.SPIKE_NAV, () -> {
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(pixelParkStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.SPIKE_NAV, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    ServoUtil.calculateFlipPose(30, flipServo);
                    if (startDist == StartDist.LONG_SIDE) {
                        blueMidOff = 7;
                    }
                })
                .transition(pixelParkStates.SPIKE_NAV, () -> (!drive.isBusy()), 0)
                .state(pixelParkStates.BACKDROP)
                .onEnter(pixelParkStates.BACKDROP, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                    navToBackdrop_Place(drive, pathLong, false);
                })
                .whileState(pixelParkStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.BACKDROP, () -> {
                    int clawOffset = 15;
                    if (startDist == StartDist.LONG_SIDE) {
                        rotate = autoRotation - 400;
                        extend = autoExtension;
//                        calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                    } else {
                        extend = (autoExtension / 3);
                    }
                    calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                    encoderDrive(motorExtension, extend, 1, drive);
                    ServoUtil.openClaw(claw1);
                })
                .transition(pixelParkStates.BACKDROP, () -> !drive.isBusy(), 0)
                .state(pixelParkStates.END_POSE)
                .onEnter(pixelParkStates.END_POSE, () -> {
                    encoderDrive(motorExtension, -extend, 0.5, drive);
                    calculateFlipPose(60, flipServo);
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                })
                .whileState(pixelParkStates.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(pixelParkStates.END_POSE, () -> !drive.isBusy(), 0)
                .state(pixelParkStates.RETRACT)
                .onEnter(pixelParkStates.RETRACT, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        encoderDrive(motorRotation, -rotate, 1, drive);
                    }
                })
                .onExit(pixelParkStates.RETRACT, () -> {
                })
                .transition(pixelParkStates.RETRACT, () -> !drive.isBusy(), 0)
                .stopRunning(pixelParkStates.STOP)
                .build();
    }

    public enum cycleStates {
        INIT,
        SPIKE_NAV,
        BACKDROP,
        //        SHIFT,
        RETRACT,
        PICK1,
        PLACE1,
        //        SHIFT2,
        END_POSE,
        RETRACT2,
        STOP,
    }

    public static StateMachine<cycleStates> cycleMachine(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        StateMachine.Builder<cycleStates> builder = new StateMachine.Builder<>();
        return builder
                .state(cycleStates.INIT)
                .onEnter(cycleStates.INIT, () -> {
                })
                .transition(cycleStates.INIT, () -> true, 0)// if we want to make it delay before entering
                .state(cycleStates.SPIKE_NAV)
                .onEnter(cycleStates.SPIKE_NAV, () -> {
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(cycleStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.SPIKE_NAV, () -> {
                    ServoUtil.openClaw(HardwareConfig.claw2);
                    ServoUtil.calculateFlipPose(30, flipServo);
                    if (startDist == StartDist.LONG_SIDE) {
                        blueMidOff = 7;
                    }
                })
                .transition(cycleStates.SPIKE_NAV, () -> (!drive.isBusy()), 0)
                .state(cycleStates.BACKDROP)
                .onEnter(cycleStates.BACKDROP, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                    navToBackdrop_Place(drive, pathLong, false);
                })
                .whileState(cycleStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.BACKDROP, () -> {
                    int clawOffset = 10;
                    if (startDist == StartDist.LONG_SIDE) {
                        rotate = autoRotation - 400;
                        extend = autoExtension;
//                        calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                    } else {
                        extend = (autoExtension / 3);
                    }
                    calculateFlipPose(AutoServoPositions.flipDown - clawOffset, flipServo);
                    encoderDrive(motorExtension, extend, 1, drive);
                    ServoUtil.openClaw(claw1);
                })
                .transition(cycleStates.BACKDROP, () -> !drive.isBusy(), 0)
                .state(cycleStates.RETRACT)
                .onEnter(cycleStates.RETRACT, () -> {
                    encoderDrive(motorExtension, -extend, 0.5, drive);
                    calculateFlipPose(60, flipServo);
                    if (startDist == StartDist.LONG_SIDE) {
                        encoderDrive(motorRotation, -rotate, 1, drive);
                    }
                })
                .onExit(cycleStates.RETRACT, () -> {
                })
                .transition(cycleStates.RETRACT, () -> !drive.isBusy(), 0)
                .state(cycleStates.PICK1)
                .onEnter(cycleStates.PICK1, () -> {
                    autoRandomReliable = AutoRandom.mid;
                    pickFromSpot(drive, pathLong);
                })
                .whileState(cycleStates.PICK1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.PICK1, () -> {
//                    closeClaw(claw2);
//                    closeClaw(claw1);
                })
                .transition(cycleStates.PICK1, () -> !drive.isBusy(), 0)
                .state(cycleStates.PLACE1)
                .onEnter(cycleStates.PLACE1, () -> {
                    AutoServoPositions.flipUp = 30;
                    forwardOffset = 0;
                    navToBackdrop_Place(drive, pathLong, true);
                })
                .whileState(cycleStates.PLACE1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.PLACE1, () -> {
                    forwardOffset = 0;
                    ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    rotate = autoRotation - 400;
                    extend = autoExtension;
                    encoderDrive(motorRotation, rotate, 1, drive);
                    encoderDrive(motorExtension, extend, 1, drive);
                    openClaw(claw1);
                    openClaw(claw2);
                })
                .transition(cycleStates.PLACE1, () -> !drive.isBusy(), 0)
                .state(cycleStates.END_POSE)
                .onEnter(cycleStates.END_POSE, () -> {
                    encoderDrive(motorExtension, -extend, 0.5, drive);
                    calculateFlipPose(60, flipServo);
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                })
                .whileState(cycleStates.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(cycleStates.END_POSE, () -> !drive.isBusy(), 0)
                .state(cycleStates.RETRACT2)
                .onEnter(cycleStates.RETRACT2, () -> {
                    encoderDrive(motorRotation, -rotate, 1, drive);
                })
                .onExit(cycleStates.RETRACT2, () -> {
                })
                .transition(cycleStates.RETRACT2, () -> !drive.isBusy(), 0)
                .stopRunning(cycleStates.STOP)
                .build();
    }
}
