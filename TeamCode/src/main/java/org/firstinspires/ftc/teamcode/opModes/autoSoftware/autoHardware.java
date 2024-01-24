package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam2_N;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.StartDist;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Trajectories.backdrop.ShiftTrajectories;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.StartPose;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.camera.VPObjectDetect;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//config can be enabled to change variables in real time through FTC Dash
//@Config
public class autoHardware extends HardwareConfig {
    //    public static OpenCvWebcam webcam; // the webcam public we are using
    public static Pose2d START_POSE = new Pose2d(0, 0, 0); // the start pose
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
    public static VisionPortal visionPortal = null; // vision portal for the webcam
    public static VPObjectDetect objProcessor = null; // april tag processor for the vision portal
    public static AprilTagProcessor aprilTagProcessor = null; // april tag processor for the vision portal

    public autoHardware(LinearOpMode opmode) {
        super(opmode);
    } // constructor

    public void initAuto(HardwareMap ahwMap, LinearOpMode myOpMode, boolean cycling) {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareMap = ahwMap; // hardware map initialization
        HardwareConfig.init(ahwMap, true); // hardware config initialization
        objProcessor = new VPObjectDetect(StartPose.alliance);
//        if (aprilTagProcessor == null && cycling == true) {
//            aprilTagProcessor = new AprilTagProcessor.Builder()
//                    .setLensIntrinsics(972.571, 972.571, 667.598, 309.012)
//                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                    .setDrawAxes(false)
//                    .setDrawTagOutline(true)
//                    .setDrawTagID(true)
//                    .build();
//        }
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cam2_N))
                .setCameraResolution(new Size(1280, 720))
                .addProcessors(objProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(objProcessor, 0); // start the camera stream on FTC Dash
        timer.reset();
        ServoUtil.closeClaw(HardwareConfig.claw1);
        ServoUtil.closeClaw(HardwareConfig.claw2);
        ServoUtil.calculateFlipPose(80, flipServo);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); // set the lights to green
        LEDcolor = "GREEN";
        telemetry.update();
        if (myOpMode.isStopRequested()) {
            return;
        }
        Sensors.ledIND(green1, red1, false);
        Sensors.ledIND(green2, red2, false);
        Sensors.ledIND(green3, red3, false);
        Sensors.ledIND(green4, red4, false);
        myOpMode.waitForStart(); // wait for the start button to be pressed
//        visionPortal.setProcessorEnabled(objProcessor, false);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK); // set the lights to the blink pattern
        LEDcolor = "HOT_PINK";
    }

    public static void doAprilTagPoseCorrection(AprilTagProcessor processor, Telemetry telemetry, MecanumDrive drive) {

        List<AprilTagDetection> currentDetections = processor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        Pose2d pose = new Pose2d(0, 0, drive.getPoseEstimate().getHeading());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 7 || detection.id == 10) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    Pose2d aprilT = new Pose2d(15, -72, Math.toRadians(0));
                    Pose2d aprilS = new Pose2d(-15, -72, Math.toRadians(0));
                    Pose2d detectablePose;
                    if (detection.id == 7) {
                        detectablePose = aprilS;
                    } else {
                        detectablePose = aprilT;
                    }
                    pose = new Pose2d(detectablePose.getX() - detection.ftcPose.y, detectablePose.getY() - detection.ftcPose.x, drive.getPoseEstimate().getHeading());
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        if (pose.getX() != 0 && pose.getY() != 0) {
            telemetry.update();
            drive.setPoseEstimate(pose);
        }
    }

    public static void endAuto(EndPose endPose, MecanumDrive drive) {
        if (endPose != EndPose.NONE) {
            goToEndPose(endPose, drive);
        }
//        visionPortal.close();
        updatePose(drive);
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
                        startDist = StartDist.LONG_SIDE;
                        START_POSE = new Pose2d(-36, -62, Math.toRadians(redRotate));
                        return new Pose2d(-36, -62, Math.toRadians(redRotate));
                    case RIGHT:
                        startDist = StartDist.SHORT_SIDE;
                        START_POSE = new Pose2d(12, -62, Math.toRadians(redRotate));
                        return new Pose2d(12, -62, Math.toRadians(redRotate));
                }
            case BLUE:
                switch (side) {
                    case LEFT:
                        startDist = StartDist.SHORT_SIDE;
                        START_POSE = new Pose2d(12, 62, Math.toRadians(blueRotate));
                        return new Pose2d(12, 62, Math.toRadians(blueRotate));
                    case RIGHT:
                        startDist = StartDist.LONG_SIDE;
                        START_POSE = new Pose2d(-36, 62, Math.toRadians(blueRotate));
                        return new Pose2d(-36, 62, Math.toRadians(blueRotate));
                }
        }
        return new Pose2d(0, 0, 0);
    }

    // method to raise the arm with the potentiometer
    public static void raiseArm() {
        int potentBackTarget = 31;
        Sensors.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorRotation);
    }

    public static void raiseArmHigh() {
        int potentBackTarget = 41;
        Sensors.driveByPotentVal(potentBackTarget, HardwareConfig.potentiometer, HardwareConfig.motorRotation);
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
        while (motor.isBusy()) {
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
