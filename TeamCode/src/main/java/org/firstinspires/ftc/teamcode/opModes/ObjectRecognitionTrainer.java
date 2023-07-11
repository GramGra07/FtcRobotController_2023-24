package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.opModes.camera.openCV.OpenCVpipelines.scalarVals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class ObjectRecognitionTrainer extends LinearOpMode {
    OpenCvWebcam webcam;

    public String name = null;
    public static String color = null;
    public static double aspectRatio;
    public static double minWidth;
    public static double minHeight;
    public static double maxWidth;
    public static double maxHeight;
    public static double minArea;
    public static double tolerance = 0.2;
    public static double translationX;
    public static double translationY;
    public static double xDistance;
    public static double yDistance;


    public double centerX;
    public double left;
    public double right;
    public double top;
    public double bottom;
    public double centerY;
    public double height;
    public double width;
    public double xDist;
    public double botDist;
    public double middle;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        OpenCvPipeline pipeline = new ColorIsolateBound();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, HardwareConfig.cam1_N), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
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
        waitForStart();
        if (opModeIsActive()) {
            while (name == null) {
                telemetry.addData("Please enter the name of the object you are training for in FTC Dash", "");
                telemetry.update();
            }
            telemetry.clearAll();
            while (color == null) {
                telemetry.addData("Please enter the color of the object you are training for in FTC Dash", "");
                telemetry.update();
            }
            while (color != "red" && color != "yellow" && color != "blue" && color != "green") {
                telemetry.addData("Please enter a valid color", "");
                telemetry.update();
            }
            telemetry.clearAll();
            telemetry.addData("Please move the object you are training for into the frame", "4 seconds until capture");
            telemetry.update();
            wait(4000);
            aspectRatio = width/height;
            telemetry.clearAll();
            telemetry.addData("Now place the object as far back as you want it to read","4 seconds until capture");
            telemetry.update();
            wait(4000);
            minWidth = width;
            minHeight = height;
            minArea = width*height;
            telemetry.clearAll();
            telemetry.addData("Now place the object as close as you want it to read","4 seconds until capture");
            telemetry.update();
            wait(4000);
            maxWidth = width;
            maxHeight = height;
            telemetry.clearAll();
            webcam.stopStreaming();
            // translations
            while (xDistance != 0) {
                telemetry.addData("Please measure the distance from the center of the camera to the center of the object and input it in FTC Dash under xDistance", "");
                telemetry.update();
            }
            telemetry.clearAll();
            translationX = xDistance/xDist;
            while (yDistance != 0) {
                telemetry.addData("Please measure the distance from the camera to the object and input it in FTC Dash under yDistance", "");
                telemetry.update();
            }
            telemetry.clearAll();
            translationY = yDistance/botDist;
            telemetry.addData("Building file, you will create a new file with the name: " + color + name + "ObjVars.java", "");
            telemetry.update();
            wait(5000);
            while (opModeIsActive()) {
                buildFile();
            }
        }
    }

    public void buildFile(){
        telemetry.addData("package ;\nimport com.acmerobotics.dashboard.config.Config;\n@Config\npublic class " + color + name + "ObjVars {", "");
        telemetry.addData("public static double aspectRatio = " + aspectRatio + ";", "");
        telemetry.addData("public static double minWidth = " + minWidth + ";", "");
        telemetry.addData("public static double minHeight = " + minHeight + ";", "");
        telemetry.addData("public static double maxWidth = " + maxWidth + ";", "");
        telemetry.addData("public static double maxHeight = " + maxHeight + ";", "");
        telemetry.addData("public static double minArea = " + minArea + ";", "");
        telemetry.addData("public static double tolerance = " + tolerance + "; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more", "");
        telemetry.addData("public static double translationX = " + translationX + ";", "");
        telemetry.addData("public static double translationY = " + translationY + ";", "");
        telemetry.addData("}", "");
        telemetry.addData("\n\nCopy this code into the new file you created, set the package name, then press stop", "");
        telemetry.update();
    }

    public class ColorIsolateBound extends OpenCvPipeline {
        public String color = ObjectRecognitionTrainer.color;
        Mat hsv = new Mat();
        Mat hsv2 = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat end = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Scalar scalarLow, scalarHigh;
            if (color == "yellow") {
                scalarLow = new Scalar(20, 100, 100);
                scalarHigh = new Scalar(30, 255, 255);
            } else if (color == "blue") {
                scalarLow = new Scalar(90, 100, 100);
                scalarHigh = new Scalar(140, 255, 255);
            } else if (color == "green") {
                scalarLow = new Scalar(40, 100, 100);
                scalarHigh = new Scalar(75, 255, 255);
            } else {
                scalarLow = new Scalar(0, 0, 0);
                scalarHigh = new Scalar(0, 0, 0);
            }
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);//change to hsv
            Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV);
            if (!color.equals("red"))
                Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
            else {
                Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(8, 255, 255), mask1);
                Core.inRange(hsv2, new Scalar(172, 70, 50), new Scalar(180, 255, 255), mask2);
                Core.bitwise_or(mask1, mask2, end);//takes both masks and combines them
            }

            Imgproc.Canny(end, edges, 25, 50);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            int highIndex = 0;
            for (int i = 0; i < contours.size(); i++) {
                Scalar c = scalarVals("red");
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 2);
                if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                    highIndex = i;
            }
            if (boundRect.length > 0) {
                left = boundRect[highIndex].tl().x;
                right = boundRect[highIndex].br().x;
                top = boundRect[highIndex].tl().y;
                bottom = boundRect[highIndex].br().y;
                centerX = (int) (left + ((right - left) / 2));
                Imgproc.putText(input, String.valueOf(centerX), new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            height = Math.abs(top - bottom);
            width = Math.abs(right - left);
            centerX = (left + right) / 2;
            centerY = (top + bottom) / 2;
            aspectRatio = width / height;
            botDist = Math.abs(input.height() - bottom);
            middle = input.width() / 2;
            xDist = Math.abs(middle - centerX);
            if (centerX<=middle) xDist = -xDist;
            return input;
        }
    }
}