package org.firstinspires.ftc.teamcode.eocvSim;

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

public class ColorEdgeDetectionBounded extends OpenCvPipeline {
    Mat end = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();
    Mat ycrcbMat = new Mat();
    public int farLeft = 100,farRight = 600,top = 240,lowTop = 200,highTop = 430,width = 50,topLeft = 150,topRight = 500;
    public Scalar scalarLow = new Scalar(0, 0, 0), scalarHigh = new Scalar(255, 255, 255);
    public Scalar c = new Scalar(255,255,255);

//    public ColorEdgeDetectionBounded(String color) {
//        this.color = color;
//    }

    @Override
    public Mat processFrame(Mat input) {
//        EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
        // color map below
        // https://i.stack.imgur.com/gyuw4.png
//        Scalar scalarLow, scalarHigh;
        Imgproc.rectangle(input, new Point(farLeft-width, highTop), new Point(farLeft, top), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(input, new Point(topLeft, lowTop), new Point(topRight, top), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(input, new Point(farRight-width, highTop), new Point(farRight, top), new Scalar(0, 255, 0), 1);
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);//change to hsv
        Core.inRange(ycrcbMat, scalarLow, scalarHigh, end);
        Core.bitwise_and(input, input, ycrcbMat, end);
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
            centers[i] = new Point();
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        int highIndex = 0;
        for (int i = 0; i < contours.size(); i++) {
            if (((centers[i].x < 50 && centers[i].x > 100) && (centers[i].y < 430 && centers[i].y > 240)) || ((centers[i].x > 150 && centers[i].x < 500) && (centers[i].y < 240 && centers[i].y > 200)) || ((centers[i].x > 550 && centers[i].x < 600) && (centers[i].y < 430 && centers[i].y > 240))) {
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 1);
                if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                    highIndex = i;
            }
        }
        if (boundRect.length > 0) {
            double left = boundRect[highIndex].tl().x;
            double right = boundRect[highIndex].br().x;
            double top = boundRect[highIndex].tl().y;
            double bottom = boundRect[highIndex].br().y;
//            pipelineTester.left = left;
//            pipelineTester.right = right;
//            pipelineTester.top = top;
//            pipelineTester.bottom = bottom;
            int centerX = (int) (left + ((right - left) / 2));
            Imgproc.putText(input, String.valueOf(centerX), new Point(left + 7, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, c, 1);
        }
        return input;
    }
}

