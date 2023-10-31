package org.firstinspires.ftc.teamcode.opModes.auto.parkAuto;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.getStartPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

@Autonomous
//@Disabled
public class parkAutoBL extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap);
        if (isStopRequested()) return;
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(90)))
                                .build()
                );
            }
            if (StartPose.side == StartSide.LEFT){
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
                                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(90)))
                                .build()
                );
            }
            if (StartPose.side == StartSide.RIGHT){
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(90)))
                                .build()
                );
            }
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}