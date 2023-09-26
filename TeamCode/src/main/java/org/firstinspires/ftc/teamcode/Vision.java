package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

public class Vision {
    public static AprilTagProcessor aprilTag;
    public static TfodProcessor tFod;
    public static VisionPortal portal;
    public static double cameraWidth = 640;
    public static final int leftB = 1;
    public static final int midB=2;
    public static final int rightB=3;
    public static final int leftR = 4;
    public static final int midR = 5;
    public static final int rightR = 6;
    public static final int redLarge = 7;
    public static final int redSmall = 8;
    public static final int blueSmall = 9;
    public static final int blueLarge = 10;
    public static final int ourTag = 12;
    public static void initVision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        tFod = new TfodProcessor.Builder()
                .setModelAssetName("")
                .setModelLabels(new String[]{"", "", ""})
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, EOCVWebcam.cam1_N));
        builder.addProcessors(aprilTag,tFod);
        portal = builder.build();
    }
    private void telemetryTfod(OpMode myOpMode) {
        List<Recognition> currentRecognitions = tFod.getRecognitions();
        myOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            myOpMode.telemetry.addData(""," ");
            myOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            myOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            myOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }

    }
    public static void telemetryAprilTag(OpMode myOpMode) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size()>0) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    myOpMode.telemetry.addData("width",cameraWidth);
                } else {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                myOpMode.telemetry.update();
            }
        }
    }
    public static boolean searchAprilTags(int id){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size()>0) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    return true;
                }
            }
        }
        return false;
    }
    public static double extractCenter(int id){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size()>0) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    return detection.center.x;
                }
            }
        }
        return 0;
    }
    public static void getPoseFromCenter(int id){
        double cameraThird = cameraWidth/3;
        if (extractCenter(id) < cameraThird){
            autoHardware.autonomousRandom = AutoRandom.left;
        }else if (extractCenter(id) > cameraThird && extractCenter(id) < cameraThird*2){
            autoHardware.autonomousRandom = AutoRandom.mid;
        }else if (extractCenter(id) > cameraThird*2){
            autoHardware.autonomousRandom = AutoRandom.right;
        }

    }
}
