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

    public static Scalar scalarVals(String color) {//rgb scalar vals
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


    public static class EdgeDetection extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat edges = new Mat();

        @Override
        public Mat processFrame(Mat input) {
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

        String color;

        public ColorEdgeDetectionBounded(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {
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

            Mat drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            int highIndex = 0;
            for (int i = 0; i < contours.size(); i++) {
                Scalar c = scalarVals(color);
                Imgproc.drawContours(drawing, contoursPolyList, i, c);
                Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), c, 2);
                if (boundRect[i].height > highIndex) highIndex = i;
                //Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
            }
            return drawing;
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
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

    public void autoAdjust(Rect[] boundRect, int index, int width) {//takes in boundRect and index of largest rect to get the x value
        int x = boundRect[index].x;
        int left = 0;
        int right = width;
        int servoRange = 180;
        int center = (left + right) / 2;
        int servoAngle = servoRange / 2;//set at beginning //aka center
        int tolerance = 10;
        //setServo(servoAngle);
        while (true) {
            if (x < center - tolerance) {//move right
                servoAngle = servoAngle + 1;
                //setServo(servoAngle);
            } else if (x > center + tolerance) {//move left
                servoAngle = servoAngle - 1;
                //setServo(servoAngle);
            }
        }
    }
}