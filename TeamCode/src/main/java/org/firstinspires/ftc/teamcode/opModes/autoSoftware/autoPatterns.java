package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class autoPatterns {
    // contains different auto patterns for different tasks
    public static void place1Pixel(MecanumDrive drive) {
        ServoUtil.calculateFlipPose(0, flipServo);
        SpikeNav(drive);
    }

    // does two pixel and then goes to the end pose
    public static void pixelPark(MecanumDrive drive, PathLong pathLong) {
        place1Pixel(drive);
        navToBackdrop_Place(drive, true, pathLong);
    }

}
