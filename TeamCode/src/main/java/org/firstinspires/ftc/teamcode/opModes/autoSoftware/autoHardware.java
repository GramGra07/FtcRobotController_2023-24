package org.firstinspires.ftc.teamcode.opModes.autoSoftware;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam2_N;
import static org.firstinspires.ftc.teamcode.Trajectories.backdrop.BackdropTrajectories.backdropOffset;
import static org.firstinspires.ftc.teamcode.UtilClass.DriverAid.operateClawByDist;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals.extensionPIDFCo;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PIDVals.rotationPIDFCo;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PotentPositions.autoPotent;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.PotentPositions.potentiometerBase;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Enums.PresetPose;
import org.firstinspires.ftc.teamcode.Enums.StartDist;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Limits;
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

    public enum STATES {
        INIT,
        SPIKE_NAV,
        BACKDROP,
        CYCLE,
        SHIFT,
        END_POSE,
        STOP
    }

    public static STATES previousState = STATES.INIT;
    public static STATES currentState = STATES.INIT;
    public static int targetPositionSlides = 0;
    public static int targetPositionPotent = 0;

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
//        ServoUtil.closeClaw(HardwareConfig.claw1);
//        ServoUtil.closeClaw(HardwareConfig.claw2);
        ServoUtil.calculateFlipPose(80, flipServo);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); // set the lights to green
        LEDcolor = "RED";
        telemetry.update();
        if (myOpMode.isStopRequested()) {
            return;
        }
        Sensors.ledIND(green1, red1, false);
        Sensors.ledIND(green2, red2, false);
        Sensors.ledIND(green3, red3, false);
        Sensors.ledIND(green4, red4, false);
        closeClaw(HardwareConfig.claw1);
        closeClaw(HardwareConfig.claw2);
        myOpMode.waitForStart(); // wait for the start button to be pressed
        currentState = STATES.SPIKE_NAV;
        rotationPIDF.setPIDF(rotationPIDFCo.p, rotationPIDFCo.i, rotationPIDFCo.d, rotationPIDFCo.f);
        extensionPIDF.setPIDF(extensionPIDFCo.p, extensionPIDFCo.i, extensionPIDFCo.d, extensionPIDFCo.f);
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

    // shifts left or right depending on the random
    public static int fwd = 1;

    public static void shiftAuto(MecanumDrive drive) {
        if (startDist == StartDist.LONG_SIDE) {
            fwd = 5;
        }
        if (autoRandomReliable == AutoRandom.left) {
            ShiftTrajectories.leftOffset = 4;
        }
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

    // method to update the pose
    public static void updatePose(MecanumDrive drive) {
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    // method to use encoders to go to a point with encoder
    public static void encoderDrive(DcMotor motor, int position, double speed, MecanumDrive drive) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition() + (position));
        drive.update();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(speed));
        while (motor.isBusy()) {
            drive.update();
        }
        motor.setPower(0);
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
