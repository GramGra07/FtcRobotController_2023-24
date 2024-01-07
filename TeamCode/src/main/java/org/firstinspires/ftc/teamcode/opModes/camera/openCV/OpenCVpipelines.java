package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.firstinspires.ftc.teamcode.UtilClass.ScalarUtil.fetchScalar;
import static org.firstinspires.ftc.teamcode.UtilClass.ScalarUtil.scalarVals;
import static org.opencv.core.CvType.CV_8U;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars.blueconeObjVars;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars.redconeObjVars;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars.redpropObjVars;
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

    public static class EdgeDetection extends OpenCvPipeline {

        Mat gray = new Mat();
        Mat edges = new Mat();

        @Override
        public Mat processFrame(Mat input) {
//            EOCVWebcam.pipelineName = "Edge Detection";
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(gray, edges, 50, 100);
            return edges;
        }
    }

    public static class ColorDetection extends OpenCvPipeline {//isolation of color
        Mat ycrcbMat = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat ycrcbMat2 = new Mat();
        Mat end = new Mat();

        String color;

        public ColorDetection(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {
//            EOCVWebcam.pipelineName = "Color Detection";
            Scalar scalarLow, scalarHigh;
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);//change to ycrcbMat
            if (!color.equals("red")) {
                scalarLow = fetchScalar("l", color, 0);
                scalarHigh = fetchScalar("h", color, 0);
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end);//detect color, output to end
            } else {
                Core.inRange(ycrcbMat, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1);
                Core.inRange(ycrcbMat2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2);
                Core.bitwise_or(mask1, mask2, end);//takes both masks and combines them
            }
            return end;
        }
    }

    public static class ColorEdgeDetection extends OpenCvPipeline {
        Mat edges = new Mat();
        Mat ycrcbMat = new Mat();
        Mat mask1 = new Mat();
        Mat ycrcbMat2 = new Mat();
        Mat mask2 = new Mat();
        Mat end = new Mat();
        String color;

        public ColorEdgeDetection(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {

//            EOCVWebcam.pipelineName = "Color Edge Detection";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
            Scalar scalarLow, scalarHigh;
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);//change to ycrcbMat
            if (!color.equals("red")) {
                scalarLow = fetchScalar("l", color, 0);
                scalarHigh = fetchScalar("h", color, 0);
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end);//detect color, output to end
            } else {
                Core.inRange(ycrcbMat, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1);
                Core.inRange(ycrcbMat2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2);
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
        Mat ycrcbMat = new Mat();
        Mat ycrcbMat2 = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();

        String color;

        public ColorEdgeDetectionBounded(String color) {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input) {
//            EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
            // color map below
            // https://i.stack.imgur.com/gyuw4.png
            Scalar scalarLow, scalarHigh;
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);//change to ycrcbMat
            Imgproc.cvtColor(input, ycrcbMat2, Imgproc.COLOR_RGB2YCrCb);
            if (!color.equals("red")) {
                scalarLow = fetchScalar("l", color, 0);
                scalarHigh = fetchScalar("h", color, 0);
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end);//detect color, output to end
            } else {
                Core.inRange(ycrcbMat, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1);
                Core.inRange(ycrcbMat2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2);
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
//            EOCVWebcam.pipelineName = "White Dot Detection";
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
//            EOCVWebcam.whiteDots = white_dots.size();
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
//            EOCVWebcam.pipelineName = "Black Dot Detection";
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
//                    EOCVWebcam.blackDotCenterX = center.x;
//                    EOCVWebcam.blackDotCenterY = center.y;
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
//            EOCVWebcam.blackDots = contours.size();
            Imgproc.putText(input, String.valueOf(contours.size()), new Point(input.width() / 16, input.height() / 6), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, scalarVals("green"), 2);
            return input;
        }
    }

    public static class RecognizeObject extends OpenCvPipeline {
        String color;
        String obj;
        String name;
        double aspectRatio;
        double xTranslation;
        double yTranslation;

        double tolerance;
        double minWidth;
        double minHeight;
        double minArea;
        double maxWidth;
        double maxHeight;
        double maxArea;
        double left;
        double right;
        double top;
        double bottom;
        int centerX;

        public RecognizeObject(String color, String obj) { // both uncaps
            this.color = color;
            this.obj = obj;
            this.name = color + obj;

            switch (name) {
                case "redcone":
                    aspectRatio = redconeObjVars.aspectRatio;
                    break;
                case "bluecone":
                    aspectRatio = blueconeObjVars.aspectRatio;
                    break;
                case "redprop":
                    aspectRatio = redpropObjVars.aspectRatio;
                    break;
                default:
                    aspectRatio = 0;
                    break;
            }

        }

        Mat ycrcbMat = new Mat();
        Mat ycrcbMat2 = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat end = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {
//            EOCVWebcam.pipelineName = "Recognize Object";
            Scalar scalarLow, scalarHigh;
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);//change to ycrcbMat
            Imgproc.cvtColor(input, ycrcbMat2, Imgproc.COLOR_RGB2YCrCb);
            if (!color.equals("red")) {
                scalarLow = fetchScalar("l", color, 0);
                scalarHigh = fetchScalar("h", color, 0);
                Core.inRange(ycrcbMat, scalarLow, scalarHigh, end);//detect color, output to end
            } else {
                Core.inRange(ycrcbMat, fetchScalar("l", color, 1), fetchScalar("h", color, 1), mask1);
                Core.inRange(ycrcbMat2, fetchScalar("l", color, 2), fetchScalar("h", color, 2), mask2);
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
                if (boundRect[i].area() > varConfig.minRectArea) {
                    Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 2);
                }
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
                    if (boundRect[i].area() > varConfig.minRectArea) {
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
            }
            if (boundRect.length > 0) {
                left = boundRect[lIndex].tl().x;
                right = boundRect[rIndex].br().x;
                top = boundRect[tIndex].tl().y;
                bottom = boundRect[bIndex].br().y;
                centerX = (int) (left + ((right - left) / 2));
                //Imgproc.putText(input, String.valueOf(centerX), new Point(middle + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals("white"), 1);
            }
            double height = Math.abs(top - bottom);
            double width = Math.abs(right - left);
            double centerX = (left + right) / 2;
            double centerY = (top + bottom) / 2;
            double newAspectRatio = width / height;
            // place if other object here
            switch (name) {
                case "redcone":
                    tolerance = redconeObjVars.tolerance;
                    minWidth = redconeObjVars.minWidth;
                    minHeight = redconeObjVars.minHeight;
                    minArea = redconeObjVars.minArea;
                    maxWidth = redconeObjVars.maxWidth;
                    maxHeight = redconeObjVars.maxHeight;
                    maxArea = redconeObjVars.maxArea;
                    break;
                case "bluecone":
                    tolerance = blueconeObjVars.tolerance;
                    minWidth = blueconeObjVars.minWidth;
                    minHeight = blueconeObjVars.minHeight;
                    minArea = blueconeObjVars.minArea;
                    maxWidth = blueconeObjVars.maxWidth;
                    maxHeight = blueconeObjVars.maxHeight;
                    maxArea = blueconeObjVars.maxArea;
                    break;
                case "redprop":
                    tolerance = redpropObjVars.tolerance;
                    minWidth = redpropObjVars.minWidth;
                    minHeight = redpropObjVars.minHeight;
                    minArea = redpropObjVars.minArea;
                    maxWidth = redpropObjVars.maxWidth;
                    maxHeight = redpropObjVars.maxHeight;
                    maxArea = redpropObjVars.maxArea;
                    break;
                default:
                    tolerance = 0.1;
                    minWidth = 0;
                    minHeight = 0;
                    minArea = 0;
                    maxWidth = 0;
                    maxHeight = 0;
                    maxArea = 0;
                    break;
            }

            Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
            telemetry.addData("New Aspect Ratio", newAspectRatio);
            telemetry.addData("Aspect Ratio", aspectRatio);
            telemetry.addData("Width", width);
            telemetry.addData("Height", height);
            telemetry.addData("Center X", centerX);
            telemetry.addData("Center Y", centerY);
            // get recognitions translation
            if ((minWidth <= width && minHeight <= height) && (maxWidth >= width && maxHeight >= height) && (minArea <= width * height && width * height <= maxArea)) {
                if (aspectRatio + tolerance >= newAspectRatio && aspectRatio - tolerance <= newAspectRatio) {
                    //should be a cone
                    Imgproc.line(input, new Point(centerX - 10, centerY), new Point(centerX + 10, centerY), scalarVals("green"));
                    Imgproc.line(input, new Point(centerX, centerY - 10), new Point(centerX, centerY + 10), scalarVals("green"));
                    Imgproc.rectangle(input, new Point(left, top), new Point(right, bottom), scalarVals("green"), 2);
                    double botDist = Math.abs(input.height() - bottom);
                    int middle = input.width() / 2;
                    double xDist = Math.abs(middle - centerX);
                    if (centerX <= middle) xDist = -xDist;
                    // add if other object here
                    switch (name) {
                        case "redcone":
                            xTranslation = xDist * redconeObjVars.translationX;
                            yTranslation = botDist * redconeObjVars.translationY;
                            break;
                        case "bluecone":
                            xTranslation = xDist * blueconeObjVars.translationX;
                            yTranslation = botDist * blueconeObjVars.translationY;
                            break;
                        case "redprop":
                            xTranslation = xDist * redpropObjVars.translationX;
                            yTranslation = botDist * redpropObjVars.translationY;
                            break;
                        default:
                            xTranslation = 0;
                            yTranslation = 0;
                            break;
                    }
                    Imgproc.line(input, new Point(middle, 0), new Point(middle, input.height()), scalarVals("yellow"), 1);
                    Imgproc.line(input, new Point(0, input.height() / 2), new Point(input.width(), input.height() / 2), scalarVals("yellow"), 1);
                }
            }
            Imgproc.line(input, new Point(input.width() / 2, 0), new Point(input.width() / 2, input.height()), scalarVals("yellow"), 1);
            Imgproc.line(input, new Point(0, input.height() / 2), new Point(input.width(), input.height() / 2), scalarVals("yellow"), 1);
//            EOCVWebcam.centerX = centerX;
//            EOCVWebcam.centerY = centerY;
            telemetry.addData("x", centerX);
            telemetry.addData("y", centerY);
            telemetry.update();
            if (centerX <= 106) {
                Imgproc.putText(input, "1", new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals(color), 2);
            }
            if (centerX > 106 && centerX <= 112) {
                Imgproc.putText(input, "2", new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals(color), 2);
            }
            if (centerX > 112 && centerX <= 320) {
                Imgproc.putText(input, "3", new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, scalarVals(color), 2);

            }
            return input;
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
//            EOCVWebcam.pipelineName = "Sample Pipeline";
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