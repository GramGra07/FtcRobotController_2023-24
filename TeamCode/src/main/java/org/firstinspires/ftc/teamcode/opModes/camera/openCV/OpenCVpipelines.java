package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.minConfidence;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.ObjectDetection.TFODBuilder;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.ObjectDetection.TensorObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class OpenCVpipelines {

    public static Scalar scalarVals(String color) {//rgb scalar vals
        if (color == "yellow") {
            return new Scalar(255, 255, 0);
        } else if (color == "blue") {
            return new Scalar(0, 0, 255);
        } else if (color == "green") {
            return new Scalar(0, 255, 0);
        } else if (color == "red") {
            return new Scalar(255, 0, 0);
        } else if (color == "black") {
            return new Scalar(0, 0, 0);
        }
        return new Scalar(0, 0, 0);
    }

    public static class EdgeDetection extends OpenCvPipeline {

        Mat gray = new Mat();
        Mat edges = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Edge Detection";
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(gray, edges, 50, 100);
            return edges;
        }
    }

    public static class ColorDetection extends OpenCvPipeline {//isolation of color
        Mat hsv = new Mat();
        Mat mask1, mask2 = new Mat();
        Mat end = new Mat();

        String color;

        public ColorDetection(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Color Detection";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
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
            if (!color.equals("red"))
                Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
            if (color == "red") {
                Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(8, 255, 255), mask1);
                Core.inRange(hsv, new Scalar(172, 70, 50), new Scalar(180, 255, 255), mask2);
                Core.bitwise_or(mask1, mask2, end);//takes both masks and combines them
            }
            return end;
        }
    }

    public static class ColorEdgeDetection extends OpenCvPipeline {
        Mat edges = new Mat();
        Mat hsv = new Mat();
        Mat mask1, mask2 = new Mat();
        Mat end = new Mat();
        String color;

        public ColorEdgeDetection(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {

            HardwareConfig.pipelineName = "Color Edge Detection";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
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
            if (!color.equals("red"))
                Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
            if (color == "red") {
                Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(8, 255, 255), mask1);
                Core.inRange(hsv, new Scalar(172, 70, 50), new Scalar(180, 255, 255), mask2);
                Core.bitwise_or(mask1, mask2, end);//takes both masks and combines them
            }
            Imgproc.Canny(end, edges, 25, 50);
            return edges;
        }
    }

    public static class ColorEdgeDetectionBounded extends OpenCvPipeline {
        Mat end = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();
        Mat hsv = new Mat();
        Mat mask1, mask2 = new Mat();
        Mat drawing = new Mat();

        String color;

        public ColorEdgeDetectionBounded(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Color Edge Detection Bounded";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
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
            if (!color.equals("red"))
                Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
            if (color == "red") {
                Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(8, 255, 255), mask1);
                Core.inRange(hsv, new Scalar(172, 70, 50), new Scalar(180, 255, 255), mask2);
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
            drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            int highIndex = 0;
            for (int i = 0; i < contours.size(); i++) {
                Scalar c = scalarVals(color);
                Imgproc.drawContours(drawing, contoursPolyList, i, c);
                Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), c, 2);
                if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                    highIndex = i;
            }
            if (boundRect.length > 0) {
                double left = boundRect[highIndex].tl().x;
                double right = boundRect[highIndex].br().x;
                double top = boundRect[highIndex].tl().y;
                double bottom = boundRect[highIndex].br().y;
                pipelineTester.left = left;
                pipelineTester.right = right;
                pipelineTester.top = top;
                pipelineTester.bottom = bottom;
                int centerX = (int) (left + ((right - left) / 2));
                Imgproc.putText(drawing, String.valueOf(centerX), new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals(color), 1);
            }
            return drawing;
        }
    }

    public static class WhiteDotDetection extends OpenCvPipeline {
        Mat blur = new Mat();
        Mat gray = new Mat();
        Mat thresh = new Mat();
        Mat hierarchy = new Mat();
        double min_area = 0.1;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.medianBlur(input, blur, 5);
            Imgproc.cvtColor(blur, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(gray, thresh, 200, 255, Imgproc.THRESH_BINARY);
            List<MatOfPoint> contours = new ArrayList<>();
            List<Object> white_dots = new ArrayList<>();
            Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (Object c : contours) {
                double area = Imgproc.contourArea((Mat) c);
                if (area > this.min_area) {
                    Imgproc.drawContours(input, contours, -1, scalarVals("green"), 2);
                    white_dots.add(c);
                }
            }
            HardwareConfig.whiteDots = white_dots.size();
            Imgproc.putText(input, String.valueOf(white_dots.size()), new Point(input.width() / 16, input.height() / 6), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, scalarVals("green"), 2);
            return input;
        }
    }

    public static class BlackDotDetection extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat circles = new Mat();
        Mat mask = new Mat();
        Mat thresh = new Mat();
        Mat masked = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.medianBlur(gray, gray, 5);
            Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                    0.1,
                    100.0, 30.0, 1, 100);
            Mat mask = new Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0));
            if (circles.cols() > 0) {
                for (int x = 0; x < circles.cols(); x++) {
                    double[] c = circles.get(0, x);
                    Point center = new Point(Math.round(c[0]), Math.round(c[1]));
                    int radius = (int) Math.round(c[2]);
                    Imgproc.circle(mask, center, radius, new Scalar(255, 255, 255), -1, 8, 0);
                }
            }
            input.copyTo(masked, mask);
            Imgproc.threshold(mask, thresh, 1, 255, Imgproc.THRESH_BINARY);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (contours.size() > 0) {
                for (int c = 0; c < contours.size(); c++) {
                    Imgproc.rectangle(input, Imgproc.boundingRect(contours.get(c)).tl(), Imgproc.boundingRect(contours.get(c)).br(), scalarVals("green"), 2);
                }
            }
            HardwareConfig.blackDots = contours.size();
            Imgproc.putText(input, String.valueOf(contours.size()), new Point(input.width() / 16, input.height() / 6), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, scalarVals("green"), 2);
            return input;
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Sample Pipeline";
            int x = input.width() / 2;
            int y = input.height() / 2;
            //vert middle line
            Imgproc.line(input, new Point(x, y - 10), new Point(x, y + 10), scalarVals("red"), 2);
            //horiz middle line
            Imgproc.line(input, new Point(x - 10, y), new Point(x + 10, y), scalarVals("red"), 2);
            //Imgproc.line(input, new Point(input.rows(), input.cols()), new Point(0, 0), scalarVals("red"), 2);
            return input;
        }
    }

    public static class xmlObjectDetection extends OpenCvPipeline {
        CascadeClassifier data = new CascadeClassifier("converted_tflite/model.tflite");
        MatOfRect rectangles = new MatOfRect();
        Vision vision = new Vision(null);

        @Override
        public Mat processFrame(Mat input) {
            data.detectMultiScale(input, rectangles);
            vision.drawRectangles(input, (List<Rect>) rectangles);
            for (Rect rect : rectangles.toArray()) {
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height),
                        scalarVals("green"), 2);
            }
            return input;
        }
    }

    public static class ObjectDetection extends OpenCvPipeline {
        HardwareMap ahwMap;
        TensorObjectDetector tfod = new TFODBuilder(ahwMap, "model.tflite", "Label 1", "Label 2").build();

        Mat mat = new Mat();
        MatOfRect faces = new MatOfRect();

        public ObjectDetection(HardwareMap ahwMap) throws IOException {
            ahwMap = this.ahwMap;
        }

        @Override
        public Mat processFrame(Mat input) {
            tfod.recognize(input);
            return null;
        }
    }

    public static class OBJDetect extends OpenCvPipeline {
        int numClasses = 2;
        int batchSize = 1;
        private Interpreter tflite;
        Mat resizedFrame = new Mat();

        public OBJDetect(String modelPath) {
            try {
                tflite = new Interpreter(new File(modelPath));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        @Override
        public Mat processFrame(Mat frame) {
            Imgproc.resize(frame, resizedFrame, new Size(frame.width(), frame.height()));
            Imgproc.cvtColor(resizedFrame, resizedFrame, Imgproc.COLOR_RGBA2RGB);
            resizedFrame.convertTo(resizedFrame, CvType.CV_32F, 1.0 / 255.0);
            float[] inputArray = new float[frame.width() * frame.height() * frame.channels()];
            resizedFrame.get(0, 0, inputArray);
            float[][] outputArray = new float[batchSize][numClasses];
            try {
                tflite.run(inputArray, outputArray);
            } catch (Exception e) {
                e.printStackTrace();
            }
            List<DetectionResult> detectionResults = new ArrayList<>();
            int numDetections = outputArray[0].length;
            for (int i = 0; i < numDetections; i++) {
                float[] detection = outputArray[i];
                float x = detection[0];
                float y = detection[1];
                float w = detection[2];
                float h = detection[3];
                Rect boundingBox = new Rect((int) x, (int) y, (int) w, (int) h); // Replace with actual values
                String classLabel = "object"; // Replace with actual class label
                float confidenceScore = 0.9f; // Replace with actual confidence score
                DetectionResult result = new DetectionResult(boundingBox, classLabel, confidenceScore);
                detectionResults.add(result);
            }
            for (DetectionResult result : detectionResults) {
                Rect bbox = DetectionResult.getBoundingBox();
                String label = result.getClassLabel();
                float score = result.getConfidenceScore();
                if (score > minConfidence) {
                    Imgproc.rectangle(frame, bbox.tl(), bbox.br(), OpenCVpipelines.scalarVals("green"), 2);
                    Imgproc.putText(frame, label + ": " + score, bbox.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, OpenCVpipelines.scalarVals("green"), 2);
                }
            }
            return frame;
        }
    }

    public static class ObjectDetectionPipeline extends OpenCvPipeline {
        public static final String LABEL_PATH = "labels.txt";
        public static final String MODEL_PATH = "model.tflite";
        public static final String FOLDER = "/sdcard/FIRST/tflitemodels/";

        @Override
        public Mat processFrame(Mat input) {
            //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            Net net = Dnn.readNetFromTensorflow(FOLDER+MODEL_PATH, FOLDER+LABEL_PATH);
            net.setInput(Dnn.blobFromImage(input, 1.0, new Size(300, 300), new Scalar(127.5, 127.5, 127.5), true, false));
            Mat detections = net.forward();
            int numDetections = detections.size(2);
            for (int i = 0; i < numDetections; i++) {
                double confidence = detections.get(0, i)[2];
                if (confidence > 0.5) { // Set a minimum confidence threshold for detection
                    int classId = (int) detections.get(0, i)[1];
                    int left = (int) (detections.get(0, i)[3] * input.cols());
                    int top = (int) (detections.get(0, i)[4] * input.rows());
                    int right = (int) (detections.get(0, i)[5] * input.cols());
                    int bottom = (int) (detections.get(0, i)[6] * input.rows());

                    // Draw bounding box and label on the frame
                    Imgproc.rectangle(input, new Point(left, top), new Point(right, bottom), new Scalar(0, 255, 0), 2);
                    String label = "Class" + classId;
                    Imgproc.putText(input, label, new Point(left, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(input, label + ": " + confidence, new Point(left, top - 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(0, 255, 0), 2);
                }
            }
            return input;
        }
    }
}