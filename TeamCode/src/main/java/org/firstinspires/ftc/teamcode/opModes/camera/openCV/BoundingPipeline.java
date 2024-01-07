package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.teleOp.BoundingBoxTuner;
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

//@Config
public class BoundingPipeline extends OpenCvPipeline {
    Mat end = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();
    Mat ycrcbMat = new Mat();
    public Scalar c = new Scalar(255, 0, 0);
    public Alliance alliance;

    public static Scalar scalarLow, scalarHigh;
    public static double[] boundingBoxCenter = new double[] {};
    public static double[] boundingBoxRight = new double[] {};

    @Override
    public Mat processFrame(Mat input) {
//        EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
        // color map below
        // https://i.stack.imgur.com/gyuw4.png
        if (alliance == Alliance.RED) {
            scalarLow = new Scalar(0, 147, 0);
            scalarHigh = new Scalar(255, 255, 255);
        } else if (alliance == Alliance.BLUE) {
            //todo change to blue
            scalarLow = new Scalar(0, 0, 141);
            scalarHigh = new Scalar(255, 255, 255);
        }
        Imgproc.rectangle(input, new Point(BoundingBoxTuner.xPoints[0],BoundingBoxTuner.yPoints[0]),new Point(BoundingBoxTuner.xPoints[1],BoundingBoxTuner.yPoints[1]),new Scalar(0,255,0));
        Imgproc.rectangle(input, new Point(BoundingBoxTuner.xPoints[2],BoundingBoxTuner.yPoints[2]), new Point(BoundingBoxTuner.xPoints[2],BoundingBoxTuner.yPoints[2]),new Scalar(0,255,0));
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
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 1);
                double centerX = boundRect[i].tl().x + ((boundRect[i].br().x - boundRect[i].tl().x) / 2);
                double centerY = boundRect[i].br().y + ((boundRect[i].tl().y - boundRect[i].br().y) / 2);
                Imgproc.line(input, new Point(centerX - 5, centerY), new Point(centerX + 5, centerY), new Scalar(0, 255, 200));
                Imgproc.line(input, new Point(centerX, centerY - 5), new Point(centerX, centerY + 5), new Scalar(0, 255, 200));
            if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                highIndex = i;
        }
        if (boundRect[highIndex].br().x<320){
            // left
            boundingBoxCenter = new double[]{ boundRect[highIndex].tl().x, boundRect[highIndex].br().x, boundRect[highIndex].tl().y, boundRect[highIndex].br().y};
        }else if (boundRect[highIndex].tl().x>320){
            boundingBoxRight = new double[]{boundRect[highIndex].tl().x, boundRect[highIndex].br().x, boundRect[highIndex].tl().y, boundRect[highIndex].br().y};
        }
        return input;
    }
}