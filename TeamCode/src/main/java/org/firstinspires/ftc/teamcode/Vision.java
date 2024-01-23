package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.aprilTagProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class Vision {
    public static AprilTagProcessor aprilTag;
    public static TfodProcessor tFod;
    public static VisionPortal portal;
    public static double cameraWidth = 640;
    public static final int leftB = 1;
    public static final int midB = 2;
    public static final int rightB = 3;
    public static final int leftR = 4;
    public static final int midR = 5;
    public static final int rightR = 6;
    public static final int redLarge = 7;
    public static final int redSmall = 8;
    public static final int blueSmall = 9;
    public static final int blueLarge = 10;
    public static final int ourTag = 12;
    public static final String baseAsset = "CenterStage.tflite";
    public static final String[] baseLabels = {
            "Pixel"
    };

    public static void telemetryTfod(OpMode myOpMode) {
        List<Recognition> currentRecognitions = tFod.getRecognitions();
        myOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            myOpMode.telemetry.addData("", " ");
            myOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            myOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            myOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }

    }

    public static void telemetryAprilTag(OpMode myOpMode, AprilTagProcessor aprilTagProcessor) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections.size() > 0) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    myOpMode.telemetry.addData("width", cameraWidth);
                } else {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }
    }

    public static boolean searchAprilTags(int id) {
        if (aprilTag.getDetections().size() > 0) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    return true;
                }
            }
        }
        return false;
    }

    public static double extractCenter(int id) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() > 0) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    return detection.center.x;
                }
            }
        }
        return 0;
    }

    public static void getPoseFromCenter(int id) {
        double cameraThird = cameraWidth / 3;
        if (extractCenter(id) < cameraThird) {
            autoHardware.autonomousRandom = AutoRandom.left;
        } else if (extractCenter(id) > cameraThird && extractCenter(id) < cameraThird * 2) {
            autoHardware.autonomousRandom = AutoRandom.mid;
        } else if (extractCenter(id) > cameraThird * 2) {
            autoHardware.autonomousRandom = AutoRandom.right;
        }

    }

    public static void findAprilTagsAndSetPose(MecanumDrive drive) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && currentDetections.size() == 1) {
                int id = detection.id;
                double x = detection.ftcPose.x;
                double y = detection.ftcPose.y;
                double z = detection.ftcPose.z;
                double pitch = detection.ftcPose.pitch;
                double roll = detection.ftcPose.roll;
                double yaw = detection.ftcPose.yaw;
                drive.setPoseEstimate(new Pose2d(x, y, yaw));
            }
        }
    }
}
