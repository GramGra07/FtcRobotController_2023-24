package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Limits.autoExtension;
import static org.firstinspires.ftc.teamcode.Limits.autoRotation;
import static org.firstinspires.ftc.teamcode.MathFunctions.threeFourths;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.blueMidOff;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories.getShift;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.openClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw1;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorExtension;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.startDist;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autoRandomReliable;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.currentState;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.encoderDrive;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.previousState;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.cyclePatterns.pickFromSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartDist;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.AutoServoPositions;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
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
                .transition(place1States.SPIKE_NAV, () -> !drive.isBusy())
//                .state(place1States.END_POSE)
//                .onEnter(place1States.END_POSE, () -> {
//                    calculateFlipPose(30, flipServo);
//                    goToEndPose(EndPose.StartingPosition, drive);
//                })
//                .whileState(place1States.END_POSE, () -> !drive.isBusy(), () -> {
//                    drive.update();
//                })
//                .transition(place1States.END_POSE, () -> !drive.isBusy())
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
                .state(pixelParkStates.SPIKE_NAV)
                .onEnter(pixelParkStates.SPIKE_NAV, () -> {
                    getShift();
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(pixelParkStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.SPIKE_NAV, () -> {
                    if (startDist == StartDist.SHORT_SIDE) {
//                        rotate = (autoRotation / 4) * 3;
//                        encoderDrive(motorRotation, rotate, 1, drive);
                        if (!(autoRandomReliable == AutoRandom.left && StartPose.alliance == Alliance.BLUE)) {
                            calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        }
                    } else {
                        blueMidOff = 7;
                    }
                })
                .transition(pixelParkStates.SPIKE_NAV, () -> (!drive.isBusy()))
                .state(pixelParkStates.BACKDROP)
                .onEnter(pixelParkStates.BACKDROP, () -> {
                    navToBackdrop_Place(drive, pathLong, false);
                })
                .whileState(pixelParkStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.BACKDROP, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        int offset = 0;
                        if (autoRandomReliable == AutoRandom.mid) {
                            offset = 10;
                        }
                        rotate = autoRotation - 400;
                        extend = autoExtension;
                        calculateFlipPose(AutoServoPositions.flipDown - offset, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                    }
                    if (startDist == StartDist.SHORT_SIDE) {
                        if (autoRandomReliable == AutoRandom.left && StartPose.alliance == Alliance.BLUE) {
                            calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        }
                        extend = threeFourths(autoExtension);
                    }
                    encoderDrive(motorExtension, extend, 1, drive);
                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    ServoUtil.openClaw(claw1);
                })
                .transition(pixelParkStates.BACKDROP, () -> !drive.isBusy())
//                .state(pixelParkStates.SHIFT)
//                .onEnter(pixelParkStates.SHIFT, () -> {
//                    shiftAuto(drive);
//                })
//                .whileState(pixelParkStates.SHIFT, () -> !drive.isBusy(), () -> {
//                    drive.update();
//                })
//                .onExit(pixelParkStates.SHIFT, () -> {
//                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
//                    ServoUtil.openClaw(claw1);
//                })
//                .transition(pixelParkStates.SHIFT, () -> !drive.isBusy())
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
                .transition(pixelParkStates.END_POSE, () -> !drive.isBusy())
                .state(pixelParkStates.RETRACT)
                .onEnter(pixelParkStates.RETRACT, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        encoderDrive(motorRotation, -rotate, 1, drive);
                    }
                })
                .onExit(pixelParkStates.RETRACT, () -> {
                })
                .transition(pixelParkStates.RETRACT, () -> !drive.isBusy())
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
        SHIFT2,
        END_POSE,
        RETRACT2,
        STOP,
    }

    public static StateMachine<cycleStates> cycleMachine(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        StateMachine.Builder<cycleStates> builder = new StateMachine.Builder<>();
        return builder
                .state(cycleStates.SPIKE_NAV)
                .onEnter(cycleStates.SPIKE_NAV, () -> {
                    getShift();
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(cycleStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.SPIKE_NAV, () -> {
                    if (startDist == StartDist.SHORT_SIDE) {
//                        rotate = (autoRotation / 4) * 3;
//                        encoderDrive(motorRotation, rotate, 1, drive);
                        if (!(autoRandomReliable == AutoRandom.left && StartPose.alliance == Alliance.BLUE)) {
                            ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        }
                    }
                })
                .transition(cycleStates.SPIKE_NAV, () -> (!drive.isBusy()))
                .state(cycleStates.BACKDROP)
                .onEnter(cycleStates.BACKDROP, () -> {
                    navToBackdrop_Place(drive, pathLong, false);
                })
                .whileState(cycleStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.BACKDROP, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        rotate = autoRotation - 400;
                        extend = autoExtension;
                        ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                    }
                    if (startDist == StartDist.SHORT_SIDE) {
                        if (autoRandomReliable == AutoRandom.left && StartPose.alliance == Alliance.BLUE) {
                            ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        }
//                        extend = threeFourths(autoExtension);
                    }
                    encoderDrive(motorExtension, extend, 1, drive);
                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    ServoUtil.openClaw(claw1);
                })
                .transition(cycleStates.BACKDROP, () -> !drive.isBusy())
//                .state(cycleStates.SHIFT)
//                .onEnter(cycleStates.SHIFT, () -> {
//                    shiftAuto(drive);
//                })
//                .whileState(cycleStates.SHIFT, () -> !drive.isBusy(), () -> {
//                    drive.update();
//                })
//                .onExit(cycleStates.SHIFT, () -> {
//                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
//                    ServoUtil.openClaw(claw1);
//                })
//                .transition(cycleStates.SHIFT, () -> !drive.isBusy())
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
                .transition(cycleStates.RETRACT, () -> !drive.isBusy())
                .state(cycleStates.PICK1)
                .onEnter(cycleStates.PICK1, () -> {
                    ShiftTrajectories.shiftOffset = 0;
                    pickFromSpot(drive, pathLong);
                })
                .whileState(cycleStates.PICK1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.PICK1, () -> {
//                    closeClaw(claw2);
//                    closeClaw(claw1);
                })
                .transition(cycleStates.PICK1, () -> !drive.isBusy())
                .state(cycleStates.PLACE1)
                .onEnter(cycleStates.PLACE1, () -> {
                    AutoServoPositions.flipUp = 45;
                    navToBackdrop_Place(drive, pathLong, true);
                })
                .whileState(cycleStates.PLACE1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(cycleStates.PLACE1, () -> !drive.isBusy())
                .state(cycleStates.SHIFT2)
                .onEnter(cycleStates.SHIFT2, () -> {
                    drive.followTrajectorySequenceAsync(ShiftTrajectories.shiftForward(drive));
                })
                .whileState(cycleStates.SHIFT2, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.SHIFT2, () -> {
                    ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    rotate = autoRotation - 300;
                    extend = autoExtension;
                    encoderDrive(motorRotation, rotate, 1, drive);
                    encoderDrive(motorExtension, extend, 1, drive);
                    openClaw(claw1);
                    openClaw(claw2);
                })
                .transition(cycleStates.SHIFT2, () -> !drive.isBusy())
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
                .transition(cycleStates.END_POSE, () -> !drive.isBusy())
                .state(cycleStates.RETRACT2)
                .onEnter(cycleStates.RETRACT2, () -> {
                    encoderDrive(motorRotation, -rotate, 1, drive);
                })
                .onExit(cycleStates.RETRACT2, () -> {
                })
                .transition(cycleStates.RETRACT2, () -> !drive.isBusy())
                .stopRunning(cycleStates.STOP)
                .build();
    }
}
