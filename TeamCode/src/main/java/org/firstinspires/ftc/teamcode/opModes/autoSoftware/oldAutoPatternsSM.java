package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

public class oldAutoPatternsSM {

//    public static void pixelPark(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
//        motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
//        motorRotation.setPower(rotationPIDF.calculate(motorRotation.getCurrentPosition(), targetPositionPotent));
//        switch (currentState) {
//            case SPIKE_NAV:
//                if (previousState != currentState) {
//                    ServoUtil.calculateFlipPose(0, flipServo);
//                    SpikeNav(drive, pathLong);
//                    previousState = currentState;
//                } else if (!drive.isBusy()) {
//                    currentState = autoHardware.STATES.BACKDROP;
//                }
//                break;
//            case BACKDROP:
//                if (previousState != currentState) {
//                    navToBackdrop_Place(drive, false, pathLong);
//                    previousState = currentState;
//                } else if (!drive.isBusy()) {
//                    currentState = autoHardware.STATES.SHIFT;
//                }
//                break;
//            case SHIFT:
//                if (previousState != currentState) {
//                    shiftAuto(drive);
//                    previousState = currentState;
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
//        drive.update();
//    }


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
//public static void cycleAuto(MecanumDrive drive, PathLong pathLong, EndPose endPose) {
//    switch (currentState) {
//        case SPIKE_NAV:
//            if (previousState != currentState) {
//                previousState = currentState;
//                calculateFlipPose(0, flipServo);
//                SpikeNav(drive, pathLong);
//            } else if (!drive.isBusy()) {
//                currentState = autoHardware.STATES.BACKDROP;
//            }
//            break;
//        case BACKDROP:
//            if (previousState != currentState) {
//                previousState = currentState;
//                navToBackdrop_Place(drive, true, pathLong);
//            } else if (!drive.isBusy()) {
//                currentState = autoHardware.STATES.SHIFT;
//            }
//            break;
//        case SHIFT:
//            if (previousState != currentState) {
//                previousState = currentState;
//                shiftAuto(drive);
//            } else if (!drive.isBusy()) {
//                currentState = autoHardware.STATES.END_POSE;
//            }
//            break;
//        case END_POSE:
//            if (previousState != currentState) {
//                previousState = currentState;
//                if (endPose != EndPose.NONE) {
//                    goToEndPose(endPose, drive);
//                }
//                updatePose(drive);
//            } else if (!drive.isBusy()) {
//                currentState = autoHardware.STATES.STOP;
//            }
//            break;
//        case INIT:
//        case STOP:
//            break;
//    }
//    motorExtension.setPower(extensionPIDF.calculate(motorExtension.getCurrentPosition(), targetPositionSlides));
//    drive.update();
//        place1Pixel(drive, pathLong);
//        navToBackdrop_Place(drive, true, pathLong);
//        for (int i = 0; i < 1; i++) {
//            pickFromSpot(drive, pathLong);
//            navToBackdrop_Place(drive, true, pathLong);
//        }
//}
}
