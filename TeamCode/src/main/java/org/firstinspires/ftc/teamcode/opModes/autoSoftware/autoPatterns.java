package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.cyclePatterns.pickFromSpot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class autoPatterns {
    // contains different auto patterns for different tasks
    public static void place1Pixel(MecanumDrive drive, PathLong pathLong) {
        ServoUtil.calculateFlipPose(0, flipServo);
        SpikeNav(drive, pathLong);
    }

    // does two pixel and then goes to the end pose
    public static void pixelPark(MecanumDrive drive, PathLong pathLong) {
        place1Pixel(drive, pathLong);
        navToBackdrop_Place(drive, true, pathLong);
    }

    public static void cycleAuto(MecanumDrive drive, PathLong pathLong) {
        place1Pixel(drive, pathLong);
        navToBackdrop_Place(drive, true, pathLong);
        for (int i = 0; i < 1; i++) {
            pickFromSpot(drive, pathLong);
            navToBackdrop_Place(drive, true, pathLong);
        }
    }

}
