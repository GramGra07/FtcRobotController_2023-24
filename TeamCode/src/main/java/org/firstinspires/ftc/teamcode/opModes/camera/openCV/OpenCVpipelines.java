package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.opencv.core.CvType.CV_8U;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.opencv.core.Core;
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
        } else if (color == "black") {
            return new Scalar(0, 0, 0);
        }
        return new Scalar(255, 255, 255);
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
        Mat hsv2 = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();

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
                Scalar c = scalarVals(color);
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 2);
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
                Imgproc.putText(input, String.valueOf(centerX), new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals(color), 2);
            }
            return input;
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
            HardwareConfig.pipelineName = "White Dot Detection";
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
        Mat thresh = new Mat();
        Mat masked = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Black Dot Detection";
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.medianBlur(gray, gray, 5);
            Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                    0.1,
                    100.0, 30.0, 1, 100);
            Mat mask = new Mat(input.rows(), input.cols(), CV_8U, Scalar.all(0));
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

    public static class RecognizeObject extends OpenCvPipeline {
        String color;
        String obj;

        public RecognizeObject(String obj, String color) {
            this.color = color;
            this.obj = obj;
        }

        double aspectRatio = ConeObjVars.aspectRatio;

        Mat hsv = new Mat();
        Mat hsv2 = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat end = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            HardwareConfig.pipelineName = "Recognize Object";
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
                double left = boundRect[highIndex].tl().x;
                double right = boundRect[highIndex].br().x;
                double top = boundRect[highIndex].tl().y;
                double bottom = boundRect[highIndex].br().y;
                pipelineTester.left = left;
                pipelineTester.right = right;
                pipelineTester.top = top;
                pipelineTester.bottom = bottom;
                int centerX = (int) (left + ((right - left) / 2));
                Imgproc.putText(input, String.valueOf(centerX), new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            double height = Math.abs(pipelineTester.top - pipelineTester.bottom);
            double width = Math.abs(pipelineTester.right - pipelineTester.left);
            double left = pipelineTester.left;
            double right = pipelineTester.right;
            double top = pipelineTester.top;
            double bottom = pipelineTester.bottom;
            double centerX = (left + right) / 2;
            double centerY = (top + bottom) / 2;
            double newAspectRatio = width / height;

            // place if other object here
            double tolerance = ConeObjVars.tolerance;
            double minWidth = ConeObjVars.minWidth;
            double minHeight = ConeObjVars.minHeight;
            double minArea = ConeObjVars.minArea;
            double maxWidth = ConeObjVars.maxWidth;
            double maxHeight = ConeObjVars.maxHeight;
            double maxArea = ConeObjVars.maxArea;

            Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
            telemetry.addData("New Aspect Ratio", newAspectRatio);
            telemetry.addData("Aspect Ratio", aspectRatio);
            telemetry.addData("Width", width);
            telemetry.addData("Height", height);
            telemetry.addData("Center X", centerX);
            telemetry.addData("Center Y", centerY);
            if ((minWidth <= width && minHeight <= height) && (maxWidth >= width && maxHeight >= height) && (minArea <= width * height && width * height <= maxArea)) {
                if (aspectRatio + tolerance > newAspectRatio && aspectRatio - tolerance < newAspectRatio) {
                    //should be a cone
                    Imgproc.circle(input, new Point(centerX, centerY), 5, scalarVals("green"), 2);

                    double botDist = Math.abs(input.height() - bottom);
                    int middle = input.width() / 2;
                    double xDist = Math.abs(middle - centerX);
                    if (centerX<=middle) xDist = -xDist;

                    // add if other object here
                    double xTranslation = xDist * ConeObjVars.translationX;
                    double yTranslation = botDist * ConeObjVars.translationY;

                    telemetry.addData("x", xTranslation);
                    telemetry.addData("y", yTranslation);
                    // report dist away from cone
                }
            }
            telemetry.update();
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

}