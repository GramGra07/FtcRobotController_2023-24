package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;
import static org.firstinspires.ftc.teamcode.Sensors.driveByPotentVal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.CycleTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesLEFT;
import org.firstinspires.ftc.teamcode.Trajectories.SpikeNavTrajectoriesRIGHT;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.OBJDetect2;
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
    public static int extensionBackdrop = 100;
    public static Pose2d spot;
    HardwareMap hardwareMap = null;

    public static AutoRandom autonomousRandom = AutoRandom.mid;

    public autoHardware(LinearOpMode opmode) {
        super(opmode);
    }

    public void initAuto(HardwareMap ahwMap, LinearOpMode myOpMode) {
        hardwareMap = ahwMap;
        HardwareConfig.init(ahwMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
        webcam.setPipeline(new OBJDetect2(StartPose.alliance));//!can switch pipelines here
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        timer.reset();
        if (myOpMode.isStopRequested()) return;
        myOpMode.waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColor()));
    }


    public static void cycle(MecanumDrive drive) {
        updatePose(drive);
        getCycleSpot();
        for (int i = 0; i < 1; i++) {
            if (((StartPose.side == StartSide.RIGHT) && (StartPose.alliance == Alliance.RED)) || ((StartPose.side == StartSide.LEFT) && (StartPose.alliance == Alliance.BLUE))) {
                //short side
                drive.followTrajectorySequence(CycleTrajectories.cycle(drive, spot, PoseStorage.currentPose));
            } else {
                //long side
            }
        }
    }

    public static void getCycleSpot() {
        if (StartPose.alliance == Alliance.RED) {
            spot = new Pose2d(-60, 12, Math.toRadians(-90));
        } else {
            spot = new Pose2d(-60, -12, Math.toRadians(-90));
        }
    }

    public static void shiftAuto(MecanumDrive drive) {
        switch (autonomousRandom) {
            case left:
                drive.followTrajectorySequence(ShiftTrajectories.shiftLeft(drive));
                break;
            case right:
                drive.followTrajectorySequence(ShiftTrajectories.shiftRight(drive));
                break;
        }
    }

    public static void shiftBack(MecanumDrive drive) {
        switch (autonomousRandom) {
            case left:
                drive.followTrajectorySequence(ShiftTrajectories.shiftRight(drive));
                break;
            case right:
                drive.followTrajectorySequence(ShiftTrajectories.shiftLeft(drive));
                break;
        }
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

    public static void dropAndRaise() {
        int potentBackTarget = 30;
        ServoUtil.openClaw(HardwareConfig.claw2);
        Sensors.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorRotation);
        ServoUtil.calculateFlipPose(30, flipServo);
    }

    public static void extendAndPlace(MecanumDrive drive) {
        shiftAuto(drive);
        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HardwareConfig.motorExtension.setTargetPosition(extensionBackdrop);
        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HardwareConfig.motorExtension.setPower(0.5);
        while (HardwareConfig.motorExtension.isBusy()) {
        }
        HardwareConfig.motorExtension.setPower(0);

        ServoUtil.openClaw(claw1);

        HardwareConfig.motorExtension.setTargetPosition(0);
        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HardwareConfig.motorExtension.setPower(-0.5);
        while (HardwareConfig.motorExtension.isBusy()) {
        }
        HardwareConfig.motorExtension.setPower(0);
        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shiftBack(drive);
        updatePose(drive);
    }

//    public static void delayUntilTagFound(OpMode myOpMode, int tag) {
//        while (!Vision.searchAprilTags(tag)) {
//            Vision.searchAprilTags(tag);
//            Vision.telemetryAprilTag(myOpMode);
//        }
//    }

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
                updatePose(drive);
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
                updatePose(drive);
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
                updatePose(drive);
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
                updatePose(drive);
        }
    }

    public static void updatePose(MecanumDrive drive) {
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
