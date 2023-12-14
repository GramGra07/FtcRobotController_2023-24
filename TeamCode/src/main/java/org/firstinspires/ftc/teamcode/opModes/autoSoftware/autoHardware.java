package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoBase;
import static org.firstinspires.ftc.teamcode.Vision.findAprilTagsAndSetPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.OBJDetect2;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
    public static AutoRandom autoRandomReliable;
    public static VisionPortal visionPortal;
    public static AprilTagProcessor aprilTagProcessor;

    public autoHardware(LinearOpMode opmode) {
        super(opmode);
    }

    public void initAuto(HardwareMap ahwMap, LinearOpMode myOpMode) {
        hardwareMap = ahwMap;
        HardwareConfig.init(ahwMap);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();

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

    public static void getCycleSpot() {
        if (StartPose.alliance == Alliance.RED) {
            spot = new Pose2d(-50, -6, Math.toRadians(180));
        } else {
            spot = new Pose2d(-60, 12, Math.toRadians(180));
        }
    }

    public static void pickFromSpot(MecanumDrive drive){
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

    public static void shiftAuto(MecanumDrive drive) {
        switch (autoRandomReliable) {
            case left:
                drive.followTrajectorySequence(ShiftTrajectories.shiftLeft(drive));
                break;
            case right:
                drive.followTrajectorySequence(ShiftTrajectories.shiftRight(drive));
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
        calculateFlipPose(60, flipServo);
        if ((StartPose.alliance == Alliance.RED && StartPose.side == StartSide.RIGHT) || (StartPose.alliance == Alliance.BLUE && StartPose.side == StartSide.RIGHT)) {
            extendAndPlace(drive);
        }
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
        shiftAuto(drive);
        ServoUtil.openClaw(claw1);
        ServoUtil.openClaw(claw2);
//        findAprilTagsAndSetPose(drive);
    }

    public static void extendAndPlace(MecanumDrive drive) {
        int potentBackTarget = 10;
        Sensors.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorRotation);

//        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        HardwareConfig.motorExtension.setTargetPosition(extensionBackdrop);
//        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        HardwareConfig.motorExtension.setPower(0.5);
//        while (HardwareConfig.motorExtension.isBusy()) {
//        }
//        HardwareConfig.motorExtension.setPower(0);
//        HardwareConfig.motorExtension.setTargetPosition(0);
//        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        HardwareConfig.motorExtension.setPower(-0.5);
//        while (HardwareConfig.motorExtension.isBusy()) {
//        }
//        HardwareConfig.motorExtension.setPower(0);
//        HardwareConfig.motorExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        updatePose(drive);
    }

    public static void SpikeNav(MecanumDrive drive) {
        flipServoBase(flipServo);
        switch (autoHardware.autonomousRandom) {
            case left:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    if (StartPose.alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(22)
                                .turn(Math.toRadians(45))
                                .addDisplacementMarker(() -> {
                                    ServoUtil.openClaw(HardwareConfig.claw2);
                                })
                                .back(1)
                                .build()
                        );
                    } else {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(22)
                                .turn(Math.toRadians(45))
                                .addDisplacementMarker(() -> {
                                    ServoUtil.openClaw(HardwareConfig.claw2);
                                })
                                .back(1)
                                .build()
                        );
                    }
                } else {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(22)
                            .turn(Math.toRadians(45))
                            .addDisplacementMarker(() -> {
                                ServoUtil.openClaw(HardwareConfig.claw2);
                            })
                            .back(1)
                            .build());
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.left;
                break;
            case mid:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                if (StartPose.side == StartSide.LEFT) {
                    if (StartPose.alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(25)
                                .turn(Math.toRadians(-45))
                                .addDisplacementMarker(() -> {
                                    ServoUtil.openClaw(HardwareConfig.claw2);
                                })
                                .back(1)
                                .build()
                        );
                    } else {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(25)
                                .addDisplacementMarker(() -> {
                                    ServoUtil.openClaw(HardwareConfig.claw2);
                                })
                                .back(1)
                                .build());
                    }
                } else {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(24)
                            .addDisplacementMarker(() -> {
                                ServoUtil.openClaw(HardwareConfig.claw2);
                            })
                            .back(1)
                            .build());
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.mid;
                break;
            case right:
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .forward(22)
                                    .turn(Math.toRadians(-45))
                                    .addDisplacementMarker(() -> {
                                        ServoUtil.openClaw(HardwareConfig.claw2);
                                    })
                                    .back(1)
                                    .build());
                } else {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .strafeRight(14)
                            .forward(20)
                            .addDisplacementMarker(() -> {
                                ServoUtil.openClaw(HardwareConfig.claw2);
                            })
                            .back(1)
                            .build()
                    );
                }
                updatePose(drive);
                autoHardware.autoRandomReliable = AutoRandom.right;
                break;
        }
    }

    public static void updatePose(MecanumDrive drive) {
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
