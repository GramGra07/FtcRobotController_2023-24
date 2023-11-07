package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesLEFT;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesRIGHT;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.HuskyLensUtil;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.ColorEdgeDetectionBounded;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Config
public class autoHardware extends HardwareConfig {
    public static OpenCvWebcam webcam;
    public static int blueRotate = -90;
    public static int redRotate = 90;


    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
    //  public static Pose2d startPose = getStartPose(StartPose.redRight);
    public static int targetTag = 0;
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }

    public static AutoRandom autonomousRandom = AutoRandom.mid;

    public void initAuto(HardwareMap ahwMap,Alliance alliance) {
        hardwareMap = ahwMap;
        init(ahwMap);
        Vision.initVision(ahwMap);
//        EOCVWebcam.initEOCV(ahwMap,webcam);
//        HuskyLensUtil.initHuskyLens(hardwareMap,myOpMode, HuskyLens.Algorithm.FACE_RECOGNITION);
        if (alliance != null) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
            webcam.setPipeline(new ColorEdgeDetectionBounded(alliance));//!can switch pipelines here
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
            webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        myOpMode.waitForStart();
        timer.reset();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColor()));
    }

    public static void driveByPotentVal(int target, AnalogInput potent, DcMotor motor) {
        double dif = target - Sensors.getPotentVal(potent);
        double range = 1;
        // turn motor until dif > 1 or close
        while (Math.abs(dif) > range) {
            double sign = -(dif/dif);
            dif = target - Sensors.getPotentVal(potent);
            motor.setPower(sign * 0.5);
        }
        motor.setPower(0);
    }

    public static Pose2d getStartPose(Alliance alliance, StartSide side) {
        StartPose.alliance = alliance;
        StartPose.side = side;
        switch (alliance) {
            case RED:
                switch (side) {
                    case LEFT:
                        return new Pose2d(-36, -62, Math.toRadians(redRotate));
                    case RIGHT:
                        return new Pose2d(12, -62, Math.toRadians(redRotate));
                }
            case BLUE:
                switch (side) {
                    case LEFT:
                        return new Pose2d(12, 62, Math.toRadians(blueRotate));
                    case RIGHT:
                        return new Pose2d(-36, 62, Math.toRadians(blueRotate));
                }
        }
        return new Pose2d(0, 0, 0);
    }
    public static void parkAuto(MecanumDrive drive){
        if (StartPose.alliance == Alliance.BLUE) {
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .strafeLeft(48)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.BLUE) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeLeft(84)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.LEFT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(50)
                                .strafeRight(84)
                                .build()
                );
            }
        }
        if (StartPose.alliance == Alliance.RED) {
            if (StartPose.side == StartSide.RIGHT) {
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(4)
                                .strafeRight(48)
                                .build()
                );
            }
        }
    }

    public static void navToBackdrop(MecanumDrive drive) {
        switch (StartPose.alliance) {
            case RED:
                switch (StartPose.side) {
                    case LEFT:
                        drive.followTrajectorySequence(BackdropTrajectories.redLong(drive));
                        break;
                    case RIGHT:
                        drive.followTrajectorySequence(BackdropTrajectories.redShort(drive));
                        break;
                }
                break;
            case BLUE:
                switch (StartPose.side) {
                    case LEFT:
                        drive.followTrajectorySequence(BackdropTrajectories.blueShort(drive));
                        break;
                    case RIGHT:
                        drive.followTrajectorySequence(BackdropTrajectories.blueLong(drive));
                        break;
                }
                break;
        }
    }

    public static void delayUntilTagFound(OpMode myOpMode, int tag) {
        while (!Vision.searchAprilTags(tag)) {
            Vision.searchAprilTags(tag);
            Vision.telemetryAprilTag(myOpMode);
        }
    }

    public static void SpikeNav(MecanumDrive drive) {
        switch (autoHardware.autonomousRandom) {
            case left:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    if (StartPose.alliance == Alliance.RED) {
                        drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeLeftLRed(drive));
                    } else {
                        drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeLeftLBlue(drive));
                    }
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeLeftR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case mid:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            case right:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeRightL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeRightR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            default:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesLEFT.navToSpikeCenterL(drive));
                } else {
                    drive.followTrajectorySequence(SpikeNavTrajectoriesRIGHT.navToSpikeCenterR(drive));
                }
                PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}
