package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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
public class pipelinesTester extends LinearOpMode {
    OpenCvWebcam webcam;
    public String detectionColor = "green";

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new ColorDetection());//!can switch pipelines here

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

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            //telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            //telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            //telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            //telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
            if (gamepad1.a) {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
            sleep(100);
        }
    }
    public Scalar scalarVals(String color){
        Enum colorEnum = {"yellow", "blue", "green", "red"};
        Scalar yellow = new Scalar(20, 100, 100);
        Scalar blue = new Scalar(100, 100, 100);
        Scalar green = new Scalar(40, 100, 100);
        Scalar red = new Scalar(0, 0, 0);
    }

    public Mat findColor(Mat input){
        // https://www.google.com/url?sa=i&url=https%3A%2F%2Fcvexplained.wordpress.com%2F2020%2F04%2F28%2Fcolor-detection-hsv%2F&psig=AOvVaw18weJWfIk1tm-CSo6V_NMa&ust=1686778216166000&source=images&cd=vfe&ved=0CBAQjRxqFwoTCKj6vdeYwf8CFQAAAAAdAAAAABAE
        Scalar scalarLow,scalarHigh;
        String color = detectionColor;
        if (color == "yellow") {
            scalarLow = new Scalar(20, 100, 100);
            scalarHigh = new Scalar(30, 255, 255);
        } else if (color == "blue") {
            scalarLow = new Scalar(90, 100, 100);
            scalarHigh = new Scalar(140, 255, 255);
        } else if (color == "green") {
            scalarLow = new Scalar(40, 100, 100);
            scalarHigh = new Scalar(80, 255, 255);
        } else {
            scalarLow = new Scalar(0, 0, 0);
            scalarHigh = new Scalar(0, 0, 0);
        }
        Mat hsv = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        Mat end = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);//change to hsv
        if (!color.equals("red"))Core.inRange(hsv, scalarLow, scalarHigh,end);//detect color, output to end
        //if (color == "red"){
        //    Mat mask1 = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        //    Mat mask2 = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        //    Core.inRange(hsv,new Scalar(0, 70, 50), new Scalar(10, 255, 255),mask1);
        //    Core.inRange(hsv, new Scalar(170, 70, 50), new Scalar(180, 255, 255),mask2);
        //    Core.bitwise_or(mask1, mask2, end);
        //}
        return end;
    }
    public Mat findEdges(Mat input){
        Mat edges = new Mat();
        Imgproc.Canny(input, edges, 25, 50);
        return edges;
    }
    public Mat drawBounds(Mat edges){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
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

        Mat drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(120, 100,255);
            Imgproc.drawContours(drawing, contoursPolyList, i, color);
            Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
            //Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
        }
        return drawing;
    }

    class EdgeDetectionPipeline extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat edges = new Mat();
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(gray, edges, 50, 100);
            return edges;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
    class ColorDetection extends OpenCvPipeline{//isolation of color
        boolean viewportPaused;
        @Override
        public Mat processFrame(Mat input) {
               return findColor(input);
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
    class ColorEdgeDetection extends OpenCvPipeline{
        @Override
        public Mat processFrame(Mat input) {
            Mat mask = findColor(input);
            Mat edges = findEdges(mask);
            return drawBounds(edges);
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}