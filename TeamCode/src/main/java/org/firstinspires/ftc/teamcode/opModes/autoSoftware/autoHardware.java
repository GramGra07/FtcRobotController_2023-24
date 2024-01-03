package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.calculateFlipPose;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.flipServoBase;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.setServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

//config can be enabled to change variables in real time through FTC Dash
//@Config
public class autoHardware extends HardwareConfig {
    public static OpenCvWebcam webcam; // the webcam public we are using
    public static int blueRotate = -90; // final blue rotation
    public static int redRotate = 90; // final red rotation

    //default start position for RoadRunner
    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
    public static int targetTag = 0; // april tag target
    public static int extensionBackdrop = 100; // how far the arm should extend to place on backdrop
    public static Pose2d spot; // cycle position to be updated
    HardwareMap hardwareMap = null; // first initialization of the hardware map

    public static AutoRandom autonomousRandom = AutoRandom.mid; // default autonomous choice for spike mark
    public static AutoRandom autoRandomReliable; // tracker for the AutoRandom enum
    public static VisionPortal visionPortal; // vision portal for the webcam
    public static AprilTagProcessor aprilTagProcessor; // april tag processor for the vision portal

    public autoHardware(LinearOpMode opmode) {
        super(opmode);
    } // constructor

    public void initAuto(HardwareMap ahwMap, LinearOpMode myOpMode) {
        hardwareMap = ahwMap; // hardware map initialization
        HardwareConfig.init(ahwMap); // hardware config initialization
//        aprilTagProcessor = new AprilTagProcessor.Builder() // april tag processor initialization
//                .setDrawAxes(true) // draw axes on the april tag
//                .setDrawCubeProjection(false) // don't draw cube projection on the april tag
//                .setDrawTagOutline(true) // draw tag outline on the april tag
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // set the tag family to 36h11
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // set the tag library to the center stage tag library
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // set the output units to inches and degrees
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                // ... these parameters are fx, fy, cx, cy.
//                .build();
//        VisionPortal.Builder builder = new VisionPortal.Builder(); // vision portal builder initialization
//        builder.setCamera(hardwareMap.get(WebcamName.class, cam1_N)); // set the camera to webcam 1
//        builder.addProcessor(aprilTagProcessor); // add the april tag processor to the vision portal
//        visionPortal = builder.build(); // build the vision portal
//        visionPortal.setProcessorEnabled(aprilTagProcessor, false); // disable the april tag processor

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
        webcam.setPipeline(new OBJDetect2(StartPose.alliance)); // set the webcam pipeline to the OBJDetect2 pipeline
        FtcDashboard.getInstance().startCameraStream(webcam, 0); // start the camera stream on FTC Dash
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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); // set the lights to green
        timer.reset();
//        ServoUtil.calculateFlipPose(60,flipServo);
        ServoUtil.closeClaw(claw1);
        ServoUtil.closeClaw(claw2);
        if (myOpMode.isStopRequested()) return;
        myOpMode.waitForStart(); // wait for the start button to be pressed
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK); // set the lights to the blink pattern
        LEDcolor = "HOT_PINK";
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

    // shifts left or right depending on the random
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

    // method to get the start pose
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

    // method to go to the backdrop
    public static void navToBackdrop_Place(MecanumDrive drive, boolean raiseArm) {
        calculateFlipPose(60, flipServo);
        if (raiseArm) {
            raiseArm();
        }
//        if ((StartPose.alliance == Alliance.RED && StartPose.side == StartSide.RIGHT) || (StartPose.alliance == Alliance.BLUE && StartPose.side == StartSide.LEFT)) {
//            raiseArm(drive);
//        }
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
        ServoUtil.calculateFlipPose(25,flipServo);
        ServoUtil.openClaw(claw1);
        ServoUtil.openClaw(claw2);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(8)
                .build());
    }

    // method to raise the arm with the potentiometer
    public static void raiseArm() {
        int potentBackTarget = 38;
        Sensors.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorRotation);
    }

    // drive and place first pixel
    public static void SpikeNav(MecanumDrive drive) {
        switch (autoHardware.autonomousRandom) {
            case left:
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
                                .addDisplacementMarker(()->{
                                    ServoUtil.calculateFlipPose(30,flipServo);
                                })
                                .strafeLeft(10)
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
                if (StartPose.side == StartSide.LEFT) {
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .forward(22)
                                    .turn(Math.toRadians(-60))
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

    // method to update the pose
    public static void updatePose(MecanumDrive drive) {
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    // method to use encoders to go to a point with encoder
    public static void encoderDrive(DcMotor motor, int position, int countsPerInch, double speed) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition() + (position * countsPerInch));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(speed));
        while (motor.isBusy()) {}
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
