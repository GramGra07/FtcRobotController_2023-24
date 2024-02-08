package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.Limits.autoExtension;
import static org.firstinspires.ftc.teamcode.Limits.autoRotation;
import static org.firstinspires.ftc.teamcode.MathFunctions.threeFourths;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorExtension;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.startDist;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.timer;
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
                .transition(place1States.SPIKE_NAV, () -> !drive.isBusy())
                .state(place1States.END_POSE)
                .onEnter(place1States.END_POSE, () -> {
                    calculateFlipPose(30, flipServo);
                    goToEndPose(EndPose.StartingPosition, drive);
                })
                .whileState(place1States.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(place1States.END_POSE, () -> !drive.isBusy())
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
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(pixelParkStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.SPIKE_NAV, () -> {
                    if (startDist == StartDist.SHORT_SIDE) {
                        rotate = (autoRotation / 4) * 3;
                        encoderDrive(motorRotation, rotate, 1, drive);
                        ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    }
                })
                .transition(pixelParkStates.SPIKE_NAV, () -> (!drive.isBusy()))
                .state(pixelParkStates.BACKDROP)
                .onEnter(pixelParkStates.BACKDROP, () -> {
                    navToBackdrop_Place(drive, false, pathLong);
                })
                .whileState(pixelParkStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.BACKDROP, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        rotate = autoRotation - 400;
                        extend = autoExtension;
                        ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                    }
                    if (startDist == StartDist.SHORT_SIDE) {
                        extend = threeFourths(autoExtension);
                    }
                    encoderDrive(motorExtension, autoExtension, 1, drive);
                })
                .transition(pixelParkStates.BACKDROP, () -> !drive.isBusy())
                .state(pixelParkStates.SHIFT)
                .onEnter(pixelParkStates.SHIFT, () -> {
                    shiftAuto(drive);
                })
                .whileState(pixelParkStates.SHIFT, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(pixelParkStates.SHIFT, () -> {
                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    ServoUtil.openClaw(HardwareConfig.claw1);
                })
                .transition(pixelParkStates.SHIFT, () -> !drive.isBusy())
                .state(pixelParkStates.END_POSE)
                .onEnter(pixelParkStates.END_POSE, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        encoderDrive(motorExtension, -autoExtension, 0.5, drive);
                    }
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
                        timer.reset();
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
        SHIFT,
        RETRACT,
        PICK1,
        PLACE1,
        END_POSE,
        STOP,
    }

    public static StateMachine<cycleStates> cycleMachine(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        StateMachine.Builder<cycleStates> builder = new StateMachine.Builder<>();
        return builder
                .state(cycleStates.SPIKE_NAV)
                .onEnter(cycleStates.SPIKE_NAV, () -> {
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                })
                .whileState(cycleStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
//                .onExit(pixelParkStates.SPIKE_NAV, () -> {
//                    if (startDist == StartDist.SHORT_SIDE) {
//                        encoderDrive(motorExtension, autoExtension, 1, drive);
//                    }
//                })
                .transition(cycleStates.SPIKE_NAV, () -> (!drive.isBusy()))
                .state(cycleStates.BACKDROP)
                .onEnter(cycleStates.BACKDROP, () -> {
                    navToBackdrop_Place(drive, false, pathLong);
                })
                .whileState(cycleStates.BACKDROP, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.BACKDROP, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        rotate = autoRotation;
                        if (autoRandomReliable == AutoRandom.right) {
                            rotate -= 100;
                        }
                        ServoUtil.calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                        encoderDrive(motorRotation, rotate, 1, drive);
                        encoderDrive(motorExtension, autoExtension, 1, drive);
                    }
                })
                .transition(cycleStates.BACKDROP, () -> !drive.isBusy())
                .state(cycleStates.SHIFT)
                .onEnter(cycleStates.SHIFT, () -> {
                    shiftAuto(drive);
                })
                .whileState(cycleStates.SHIFT, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.SHIFT, () -> {
                    calculateFlipPose(AutoServoPositions.flipDown, flipServo);
                    ServoUtil.openClaw(HardwareConfig.claw1);
                })
                .transition(cycleStates.SHIFT, () -> !drive.isBusy())
                .state(cycleStates.RETRACT)
//                .onEnter(cycleStates.RETRACT, () -> {
//                    if (startDist == StartDist.LONG_SIDE) {
//                        timer.reset();
//                        encoderDrive(motorRotation, -rotate, 1, drive);
//                    }
//                })
//                .onExit(cycleStates.RETRACT, () -> {
//                    if (startDist == StartDist.LONG_SIDE) {
//                        encoderDrive(motorExtension, -autoExtension, 0.5, drive);
//                    }
//                })
//                .transition(cycleStates.RETRACT, () -> !drive.isBusy())
                .state(cycleStates.PICK1)
                .onEnter(cycleStates.PICK1, () -> {
                    //retract both
                    encoderDrive(motorExtension, -autoExtension, 1, drive);
                    encoderDrive(motorRotation, -rotate, 1, drive);
                    pickFromSpot(drive, pathLong);
                })
                .whileState(cycleStates.PICK1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
//                .onExit(cycleStates.PICK1, () -> {
//                    encoderDrive(motorExtension, autoExtension, 1, drive);
//                })
                .transition(cycleStates.PICK1, () -> !drive.isBusy())
                .state(cycleStates.PLACE1)
                .onEnter(cycleStates.PLACE1, () -> {
                    encoderDrive(motorExtension, autoExtension, 0.5, drive);
                    encoderDrive(motorRotation, rotate, 1, drive);
                    navToBackdrop_Place(drive, true, pathLong);
                })
                .whileState(cycleStates.PLACE1, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .onExit(cycleStates.PLACE1, () -> {
                    encoderDrive(motorExtension, -autoExtension, 1, drive);
                })
                .transition(cycleStates.PLACE1, () -> !drive.isBusy())
                .state(cycleStates.END_POSE)
                .onEnter(cycleStates.END_POSE, () -> {
                    calculateFlipPose(60, flipServo);
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                })
                .whileState(cycleStates.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(cycleStates.END_POSE, () -> !drive.isBusy())
                .state(cycleStates.RETRACT)
                .onEnter(cycleStates.RETRACT, () -> {
                    if (startDist == StartDist.LONG_SIDE) {
                        timer.reset();
                        encoderDrive(motorRotation, -rotate, 1, drive);
                    }
                })
                .onExit(cycleStates.RETRACT, () -> {
                })
                .transition(cycleStates.RETRACT, () -> !drive.isBusy())
                .stopRunning(cycleStates.STOP)
                .build();
    }

    public static void cycleAuto(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        switch (currentState) {
            case SPIKE_NAV:
                if (previousState != currentState) {
                    previousState = currentState;
                    calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.BACKDROP;
                }
                break;
            case BACKDROP:
                if (previousState != currentState) {
                    previousState = currentState;
                    navToBackdrop_Place(drive, true, pathLong);
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.SHIFT;
                }
                break;
            case SHIFT:
                if (previousState != currentState) {
                    previousState = currentState;
                    shiftAuto(drive);
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.END_POSE;
                }
                break;
            case END_POSE:
                if (previousState != currentState) {
                    previousState = currentState;
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                    updatePose(drive);
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.STOP;
                }
                break;
            case INIT:
            case STOP:
                break;
        }
//    motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
//    drive.update();
//        place1Pixel(drive, pathLong);
//        navToBackdrop_Place(drive, true, pathLong);
//        for (int i = 0; i < 1; i++) {
//            pickFromSpot(drive, pathLong);
//            navToBackdrop_Place(drive, true, pathLong);
//        }
    }

}
