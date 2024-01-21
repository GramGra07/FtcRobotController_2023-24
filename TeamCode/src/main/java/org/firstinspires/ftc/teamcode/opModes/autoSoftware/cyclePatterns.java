package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.claw2;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.spot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

public class cyclePatterns {
    // method to get the cycle spot
    public static void getCycleSpot() {
        switch (StartPose.alliance) {
            case RED:
                spot = new Pose2d(-50, -6, Math.toRadians(180));
                break;
            case BLUE:
                spot = new Pose2d(-50, 6, Math.toRadians(180));
                break;
        }
    }

    // method to pick up from the stack of pixels
    public static void pickFromSpot(MecanumDrive drive) {
        getCycleSpot();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(spot)
                .addDisplacementMarker(() -> calculateFlipPose(45, flipServo))
                .addDisplacementMarker(() -> closeClaw(claw2))
                .back(1)
                .build()
        );
    }
}
