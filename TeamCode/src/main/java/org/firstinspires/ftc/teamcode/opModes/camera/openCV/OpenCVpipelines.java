package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCVpipelines {
    public static String detectionColor;

    public static Scalar scalarVals(String color) {//rgb
        if (color == "yellow") {
            return new Scalar(255, 255, 0);
        } else if (color == "blue") {
            return new Scalar(0, 0, 255);
        } else if (color == "green") {
            return new Scalar(0, 255, 0);
        } else if (color == "red") {
            return new Scalar(255, 0, 0);
        } else {
            return new Scalar(0, 0, 0);
        }
    }

    public static Mat findColor(Mat input) {
        // https://www.google.com/url?sa=i&url=https%3A%2F%2Fcvexplained.wordpress.com%2F2020%2F04%2F28%2Fcolor-detection-hsv%2F&psig=AOvVaw18weJWfIk1tm-CSo6V_NMa&ust=1686778216166000&source=images&cd=vfe&ved=0CBAQjRxqFwoTCKj6vdeYwf8CFQAAAAAdAAAAABAE
        Scalar scalarLow, scalarHigh;
        String color = detectionColor;
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
        Mat hsv = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        Mat end = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);//change to hsv
        if (!color.equals("red"))
            Core.inRange(hsv, scalarLow, scalarHigh, end);//detect color, output to end
        if (color == "red") {
            Mat mask1 = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
            Mat mask2 = new Mat(input.cols(), input.rows(), CvType.CV_8UC3);
            Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(8, 255, 255), mask1);
            Core.inRange(hsv, new Scalar(172, 70, 50), new Scalar(180, 255, 255), mask2);
            Core.bitwise_or(mask1, mask2, end);//takes both masks and combines them
        }
        return end;
    }

    public static Mat findEdges(Mat input) {
        Mat edges = new Mat();
        Imgproc.Canny(input, edges, 25, 50);
        return edges;
    }

    public static Mat drawBounds(Mat edges) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
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

        Mat drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar c = scalarVals(detectionColor);
            Imgproc.drawContours(drawing, contoursPolyList, i, c);
            Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), c, 2);
            //Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
        }
        return drawing;
    }

    public static class EdgeDetectionPipeline extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat edges = new Mat();
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(gray, edges, 50, 100);
            return edges;
        }
    }

    public static class ColorDetection extends OpenCvPipeline {//isolation of color
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            return findColor(input);
        }
    }

    public static class ColorEdgeDetection extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat mask = findColor(input);
            return findEdges(mask);
        }
    }

    public static class ColorEdgeDetectionBounded extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat mask = findColor(input);
            Mat edges = findEdges(mask);
            return drawBounds(edges);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
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
    }
}