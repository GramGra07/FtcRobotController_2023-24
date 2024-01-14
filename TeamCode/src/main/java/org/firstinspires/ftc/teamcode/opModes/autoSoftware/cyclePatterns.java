package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.motorRotation;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.potentiometer;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.spot;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.updatePose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoPatterns.place1Pixel;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.navToBackdrop_Place;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class cyclePatterns {

    // method to grab the pixel early on the long side and only if on long side
    public static void grabPixelLongSide(MecanumDrive drive) {
        if (((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.BLUE))) {
            // long side
            pickFromSpot(drive);
            updatePose(drive);
        }
    }

    // for long side
    public static void place2Cycle(MecanumDrive drive) {
        place1Pixel(drive);
        grabPixelLongSide(drive);
        navToBackdrop_Place(drive, false);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(10).build());
        updatePose(drive);
    }

    // method to cycle in autonomous
    public static void cycle(MecanumDrive drive, Pose2d spot) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) { // simple for loop to cycle i times
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, drive.getPoseEstimate()));
            } else {
                //long side
            }
        }
    }

    // method to get the cycle spot
    public static void getCycleSpot() {
        if (StartPose.alliance == Alliance.RED) {
            spot = new Pose2d(-50, -6, Math.toRadians(180));
        } else {
            spot = new Pose2d(-60, 12, Math.toRadians(180));
        }
    }

    // method to pick up from the stack of pixels
    public static void pickFromSpot(MecanumDrive drive) {
        getCycleSpot();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(spot)
                .addDisplacementMarker(() -> Sensors.driveByPotentVal(6, potentiometer, motorRotation))
                .addDisplacementMarker(() -> calculateFlipPose(0, flipServo))
                .back(1)
                .build()
        );
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(4)
                .build()
        );
        closeClaw(claw2);
    }
}
