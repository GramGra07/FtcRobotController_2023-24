package org.firstinspires.ftc.teamcode.opModes.auto.parkAuto;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous(group = "park")
//@Disabled
public class parkAutoRL extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.RED, StartSide.LEFT));
        robot.initAuto(hardwareMap);
        if (isStopRequested()) return;
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeRight(84)
                                .build()
                );
            }
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(36, 48, Math.toRadians(90)))
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.BLUE) {
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(0)))
                                .build()
                );
            }
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeLeft(84)
                                .build()
                );
            }
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}