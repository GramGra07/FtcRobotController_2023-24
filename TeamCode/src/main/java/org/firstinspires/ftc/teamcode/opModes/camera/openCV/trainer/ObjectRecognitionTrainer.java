package org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer;


import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;
import static org.firstinspires.ftc.teamcode.UtilClass.ScalarUtil.fetchScalar;
import static org.firstinspires.ftc.teamcode.UtilClass.ScalarUtil.scalarVals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
//@Config
public class ObjectRecognitionTrainer extends LinearOpMode {
    OpenCvWebcam webcam;

    public static String name = "";
    public static String color = "";
    public double aspectRatio;
    public double minWidth;
    public double minHeight;
    public double maxWidth;
    public double maxHeight;
    public static double tolerance = 0.2;
    public double translationX;
    public double translationY;
    public static double xDistance = 0;
    public static double yDistance = 0;

    public String heightError = "";
    public String widthError = "";
    public String areaError = "";
    public String aspectError1 = "";
    public String aspectError2 = "";
    public String xError = "";
    public String yError = "";

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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
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
        telemetry.addLine("Please navigate to FTC Dashboard @ http://192.168.43.1:8080/dash");
        telemetry.addLine("Also have the camera and configuration tab open on the Dashboard");
        telemetry.addLine("Please open the ObjectRecognitionTrainer in the configuration tab, this is where you will edit the variables");
        telemetry.addLine("Press enter to save any changes made on FTC Dash");
        telemetry.addLine("Please enter the name and color of the object you are training for in FTC Dash");
        telemetry.addLine("If you just now entered name and color, please hit stop and re-start this program");
        telemetry.addLine("Note that while training, make sure that it only sees the object you want it to");
        telemetry.addLine("Start the OpMode");
        telemetry.update();
        waitForStart();
        telemetry.clearAll();
        if (opModeIsActive()) {
            if (color == "" || name == "") {
                telemetry.addData("Please enter the name and color of the object you are training for in FTC Dash", "stop the opmode and restart");
                telemetry.update();
                while (!gamepad1.square) {
                    if (isStopRequested()) {
                        return;
                    }
                }
            }
            telemetry.clearAll();
            telemetry.addData("Please move the object you are training for into the frame", "press circle on gamepad to capture");
            telemetry.update();
            while (!gamepad1.circle) {
                if (isStopRequested()) {
                    return;
                }
            }
            aspectRatio = width / height;
            telemetry.clearAll();
            telemetry.addData("Now place the object as far back as you want it to read", "press square on gamepad to capture");
            telemetry.update();
            while (!gamepad1.square) {
                if (isStopRequested()) {
                    return;
                }
            }
            minWidth = width;
            minHeight = height;
            telemetry.clearAll();
            telemetry.addData("Now place the object as close as you want it to read", "press triangle on gamepad to capture");
            telemetry.update();
            while (!gamepad1.triangle) {
                if (isStopRequested()) {
                    return;
                }
            }
            maxWidth = width;
            maxHeight = height;
            telemetry.clearAll();
            telemetry.addData("Please move object back to starting position", "press square on gamepad when complete");
            telemetry.update();
            while (!gamepad1.square) {
                if (isStopRequested()) {
                    return;
                }
            }
            telemetry.clearAll();
            // translations
            telemetry.addData("Please measure the distance from the center of the camera to the center of the object and input it in FTC Dash under xDistance", "");
            telemetry.addData("Please measure the distance from the camera to the front of the object and input it in FTC Dash under yDistance", "");
            telemetry.addLine("Press circle on gamepad to complete");
            telemetry.update();
            while (!gamepad1.circle) {
                if (isStopRequested()) {
                    return;
                }
            }
            if (botDist == 0) {
                telemetry.addData("Object too close, please move robot back", "press triangle on gamepad when done");
                telemetry.update();
                while (!gamepad1.triangle) {
                    if (isStopRequested()) {
                        return;
                    }
                }
            }
            telemetry.clearAll();
            webcam.stopStreaming();
            translationX = Math.abs(xDistance / xDist);
            translationY = yDistance / botDist;
            String fullName = color + name;
            String caseName = "\"" + fullName + "\"";
            telemetry.addLine("Building file, you will create a new java file with the name: " + fullName + "ObjVars.java");
            telemetry.update();
            sleep(1500);
            telemetry.addLine("Now running check on variables created");
            telemetry.update();
            sleep(1500);
            checkVars();
            buildFile();
            telemetry.addData("File built, please copy and paste the file into your project", "press square on gamepad when done");
            telemetry.update();
            while (!gamepad1.square) {
                if (isStopRequested()) {
                    return;
                }
            }
            telemetry.clearAll();

            telemetry.addLine("To finish setup you will add the following to the Object Recognition Pipeline");
            telemetry.addLine("In the constructor add the following lines: ");
            telemetry.addLine("case " + caseName + ":");
            telemetry.addLine("aspectRatio = " + fullName + "ObjVars.aspectRatio;");
            telemetry.addLine("break;");
            telemetry.addLine("Press circle on gamepad to continue");
            telemetry.update();
            while (!gamepad1.circle) {
                if (isStopRequested()) {
                    return;
                }
            }
            telemetry.clearAll();
            telemetry.addLine("Add the following lines under process frame and the switch statement");
            telemetry.addLine("case " + caseName + ":");
            telemetry.addLine("tolerance = " + fullName + "ObjVars.tolerance;");
            telemetry.addLine("minWidth = " + fullName + "ObjVars.minWidth;");
            telemetry.addLine("minHeight = " + fullName + "ObjVars.minHeight;");
            telemetry.addLine("minArea = " + fullName + "ObjVars.minArea;");
            telemetry.addLine("maxWidth = " + fullName + "ObjVars.maxWidth;");
            telemetry.addLine("maxHeight = " + fullName + "ObjVars.maxHeight;");
            telemetry.addLine("maxArea = " + fullName + "ObjVars.maxArea;");
            telemetry.addLine("break;");
            telemetry.addLine("Press square on gamepad to continue");
            telemetry.update();
            while (!gamepad1.square) {
                if (isStopRequested()) {
                    return;
                }
            }
            telemetry.clearAll();
            telemetry.addLine("Add the following lines under get recognitions (in a comment) and the switch statement");
            telemetry.addLine("case " + caseName + ":");
            telemetry.addLine("xTranslation = xDist * " + fullName + "ObjVars.translationX;");
            telemetry.addLine("yTranslation = botDist * " + fullName + "ObjVars.translationY;");
            telemetry.addLine("break;");
            telemetry.addData("Once done", "press triangle to continue");
            telemetry.update();
            while (!gamepad1.triangle) {
                if (isStopRequested()) {
                    return;
                }
            }
            telemetry.clearAll();
            telemetry.addLine("All finished, now just change the name in pieplineTester of whatever opmode to the color and name and it should work!");
            telemetry.addLine("Please press stop when done");
            telemetry.update();
            webcam.closeCameraDevice();
        }
    }

    public void checkVars() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Possible errors:");
        if (minHeight > maxHeight) heightError = "// possible height error minHeight>maxHeight";
        if (minWidth > maxWidth) widthError = "// possible width error minWidth>maxWidth";
        if ((minWidth * minHeight) > (maxWidth * maxHeight))
            areaError = "// possible area error minArea>maxArea";
        if (minHeight < 0) heightError = "// possible height error minHeight<0";
        if (minWidth < 0) widthError = "// possible width error minWidth<0";
        if ((minWidth * minHeight) < 0) areaError = "// possible area error minArea<0";
        double maxAspect = maxWidth / maxHeight;
        double minAspect = minWidth / minHeight;
        if (!(aspectRatio + tolerance >= maxAspect) && !(maxAspect >= aspectRatio - tolerance))
            aspectError2 = "// possible aspect ratio error, max's not in aspect";
        if (!(aspectRatio + tolerance >= minAspect) && !(minAspect >= aspectRatio - tolerance))
            aspectError1 = "// possible aspect ratio error, min's not in aspect";
        if (translationX > 100) xError = "// possible translation error, xTranslation>100";
        if (translationY > 100) yError = "// possible translation error, yTranslation>100";
    }

    public void buildFile() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.clearAll();
        telemetry.addData("Name", color + name + "ObjVars.java");
        telemetry.addLine("package ; // set this to your correct package");
        telemetry.addLine("import com.acmerobotics.dashboard.config.Config;");
        telemetry.addLine("@Config");
        telemetry.addLine("public class " + color + name + "ObjVars {");
        telemetry.addLine("public static double aspectRatio = " + aspectRatio + ";" + aspectError1 + aspectError2);
        telemetry.addLine("public static double minWidth = " + minWidth + ";" + widthError);
        telemetry.addLine("public static double minHeight = " + minHeight + ";" + heightError);
        telemetry.addLine("public static double maxWidth = " + maxWidth + ";");
        telemetry.addLine("public static double maxHeight = " + maxHeight + ";");
        telemetry.addLine("public static double minArea = minWidth * minHeight;" + areaError);
        telemetry.addLine("public static double maxArea = maxWidth * maxHeight;");
        telemetry.addLine("public static double tolerance = " + tolerance + "; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more");
        telemetry.addLine("public static double translationX = " + translationX + ";" + xError);
        telemetry.addLine("public static double translationY = " + translationY + ";" + yError);
        telemetry.addLine("}");
        telemetry.addLine("Copy this code into the new file you created and set the package name");
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
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);//change to hsv
            Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV);
            if (!color.equals("red")) {
                scalarLow = fetchScalar("l", color, 0);
                scalarHigh = fetchScalar("h", color, 0);
                Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
            } else {
                Core.inRange(hsv, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1);
                Core.inRange(hsv2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2);
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
                Scalar c = scalarVals(color);
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 2);
                if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                    highIndex = i;
            }
            int lIndex = 0;
            int rIndex = 0;
            int tIndex = 0;
            int bIndex = 0;
            if (boundRect.length > 0) {
                // find furthest middle
                for (int i = 0; i < boundRect.length; i++) {
                    if (boundRect[i].tl().x <= boundRect[lIndex].tl().x) {
                        lIndex = i;
                    }
                    if (boundRect[i].br().x >= boundRect[rIndex].br().x) {
                        rIndex = i;
                    }
                    if (boundRect[i].tl().y <= boundRect[tIndex].tl().y) {
                        tIndex = i;
                    }
                    if (boundRect[i].br().y >= boundRect[bIndex].br().y) {
                        bIndex = i;
                    }
                }
            }
            if (boundRect.length > 0) {
                left = boundRect[lIndex].tl().x;
                right = boundRect[rIndex].br().x;
                top = boundRect[tIndex].tl().y;
                bottom = boundRect[bIndex].br().y;
                centerX = (int) (left + ((right - left) / 2));
                //Imgproc.putText(input, String.valueOf(centerX), new Point(middle + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            middle = input.width() / 2;
            Imgproc.line(input, new Point(middle, 0), new Point(middle, input.height()), scalarVals("yellow"), 1);
            height = Math.abs(top - bottom);
            width = Math.abs(right - left);
            centerX = (left + right) / 2;
            centerY = (top + bottom) / 2;
            aspectRatio = width / height;
            botDist = Math.abs(input.height() - bottom);
            xDist = Math.abs(middle - centerX);
            if (centerX <= middle) xDist = -xDist;

            Imgproc.line(input, new Point(centerX, top), new Point(centerX, 0), scalarVals(color), 1);
            Imgproc.line(input, new Point(centerX, bottom), new Point(centerX, input.height()), scalarVals(color), 1);
            Imgproc.putText(input, String.valueOf(Math.abs(xDist)), new Point((centerX + middle) / 2, 15), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            Imgproc.putText(input, String.valueOf(botDist), new Point(centerX + 2, (bottom + input.height()) / 2), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            return input;
        }
    }
}