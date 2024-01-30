package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.extensionPIDF;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorExtension;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.rotationPIDF;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.currentState;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.previousState;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.shiftAuto;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.targetPositionPotent;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.targetPositionSlides;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
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
                    ServoUtil.calculateFlipPose(0, flipServo);
                    SpikeNav(drive, PathLong.NONE);
                })
                .whileState(place1States.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(place1States.SPIKE_NAV, () -> !drive.isBusy())
                .state(place1States.END_POSE)
                .onEnter(place1States.END_POSE, () -> {
                    ServoUtil.calculateFlipPose(30, flipServo);
                    goToEndPose(EndPose.StartingPosition, drive);
                })
                .whileState(place1States.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(place1States.END_POSE, () -> !drive.isBusy())
                .stopRunning(place1States.STOP)
                .build();
    }

//    public static void place1Pixel(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
//
//        switch (currentState) {
//            case SPIKE_NAV:
//                if (previousState != currentState) {
//                    ServoUtil.calculateFlipPose(0, flipServo);
//                    SpikeNav(drive, pathLong);
//                    previousState = autoHardware.STATES.SPIKE_NAV;
//                } else if (!drive.isBusy()) {
//                    currentState = autoHardware.STATES.END_POSE;
//                }
//                break;
//            case END_POSE:
//                if (previousState != currentState) {
//                    if (endPose != EndPose.NONE) {
//                        goToEndPose(endPose, drive);
//                    }
//                    previousState = currentState;
//                } else if (!drive.isBusy()) {
//                    currentState = autoHardware.STATES.STOP;
//                }
//                break;
//            case INIT:
//            case STOP:
//                break;
//        }
//        motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
//        motorRotation.setPower(rotationPIDF.calculate(motorRotation.getCurrentPosition(), targetPositionPotent));
//        drive.update();
//    }

    // does two pixel and then goes to the end pose
    public enum pixelParkStates {
        INIT,
        SPIKE_NAV,
        BACKDROP,
        SHIFT,
        END_POSE,
        STOP,
    }
    public static StateMachine<pixelParkStates> pixelParkMachine(MecanumDrive drive,PathLong pathLong,EndPose endPose) {
        StateMachine.Builder<pixelParkStates> builder = new StateMachine.Builder<>();
        return builder
                .state(pixelParkStates.SPIKE_NAV)
                .onEnter(pixelParkStates.SPIKE_NAV, () -> {
                    ServoUtil.calculateFlipPose(0, flipServo);
                    SpikeNav(drive, PathLong.NONE);
                })
                .whileState(pixelParkStates.SPIKE_NAV, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(pixelParkStates.SPIKE_NAV, () -> !drive.isBusy())
                //^same
                .state(pixelParkStates.BACKDROP)
                .onEnter(pixelParkStates.BACKDROP,()->{
                    navToBackdrop_Place(drive, false, pathLong);
                })
                .whileState(pixelParkStates.BACKDROP,()->!drive.isBusy(),()->{
                    drive.update();
                    motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
                    motorRotation.setPower(rotationPIDF.calculate(motorRotation.getCurrentPosition(), targetPositionPotent));
                })
                .transition(pixelParkStates.BACKDROP,()->!drive.isBusy())
                .state(pixelParkStates.SHIFT)
                .onEnter(pixelParkStates.SHIFT, () -> {
                    shiftAuto(drive);
                })
                .whileState(pixelParkStates.SHIFT, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(pixelParkStates.SHIFT, () -> !drive.isBusy())
                .state(pixelParkStates.END_POSE)
                .onEnter(pixelParkStates.END_POSE, () -> {
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                })
                .whileState(pixelParkStates.END_POSE, () -> !drive.isBusy(), () -> {
                    drive.update();
                })
                .transition(pixelParkStates.END_POSE, () -> !drive.isBusy())
                .stopRunning(pixelParkStates.STOP)
                .build();
    }
    public static void pixelPark(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
        motorRotation.setPower(rotationPIDF.calculate(motorRotation.getCurrentPosition(), targetPositionPotent));
        switch (currentState) {
            case SPIKE_NAV:
                if (previousState != currentState) {
                    ServoUtil.calculateFlipPose(0, flipServo);
                    SpikeNav(drive, pathLong);
                    previousState = currentState;
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.BACKDROP;
                }
                break;
            case BACKDROP:
                if (previousState != currentState) {
                    navToBackdrop_Place(drive, false, pathLong);
                    previousState = currentState;
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.SHIFT;
                }
                break;
            case SHIFT:
                if (previousState != currentState) {
                    shiftAuto(drive);
                    previousState = currentState;
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.END_POSE;
                }
                break;
            case END_POSE:
                if (previousState != currentState) {
                    if (endPose != EndPose.NONE) {
                        goToEndPose(endPose, drive);
                    }
                    previousState = currentState;
                } else if (!drive.isBusy()) {
                    currentState = autoHardware.STATES.STOP;
                }
                break;
            case INIT:
            case STOP:
                break;
        }
        drive.update();
    }

    public static void cycleAuto(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
        switch (currentState) {
            case SPIKE_NAV:
                if (previousState != currentState) {
                    previousState = currentState;
                    ServoUtil.calculateFlipPose(0, flipServo);
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
        motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
        drive.update();
//        place1Pixel(drive, pathLong);
//        navToBackdrop_Place(drive, true, pathLong);
//        for (int i = 0; i < 1; i++) {
//            pickFromSpot(drive, pathLong);
//            navToBackdrop_Place(drive, true, pathLong);
//        }
    }

}
