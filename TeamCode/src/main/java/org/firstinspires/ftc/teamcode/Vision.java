package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision {
    public static AprilTagProcessor aprilTag;
    public static VisionPortal portal;
    public static double cameraWidth = 640;
    public final int leftB = 1;
    public final int midB=2;
    public final int rightB=3;
    public final int leftR = 4;
    public final int midR = 5;
    public final int rightR = 6;
    public final int redLarge = 7;
    public final int redSmall = 8;
    public final int blueSmall = 9;
    public final int blueLarge = 10;
    private void initVision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, EOCVWebcam.cam1_N));
        builder.addProcessor(aprilTag);
        portal = builder.build();
    }
    public static void telemetryAprilTag(OpMode myOpMode) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
    public static int searchAprilTags(int id){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                return detection.id;
            }
        }
        return 0;
    }
    public static double extractCenter(int id){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        return currentDetections.get(searchAprilTags(id)).center.x;
    }
    public static void getPoseFromBearing(int id){
        double cameraThird = cameraWidth/3;
        if (extractCenter(id) < cameraThird){
            //left
        }else if (extractCenter(id) > cameraThird && extractCenter(id) < cameraThird*2){
            //middle
        }else if (extractCenter(id) > cameraThird*2){
            //right
        }
    }
}
