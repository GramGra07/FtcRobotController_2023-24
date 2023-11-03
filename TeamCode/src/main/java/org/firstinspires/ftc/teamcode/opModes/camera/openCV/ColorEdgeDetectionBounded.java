package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.firstinspires.ftc.teamcode.opModes.autoHardware.autonomousRandom;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
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
    public Scalar c = new Scalar(255, 0, 0);
    public Alliance alliance;

    public ColorEdgeDetectionBounded(Alliance color) {
        this.alliance = color;
    }
    int[] pointsX = new int[]{0, 50, 75, 250, 270, 320};
    int[] pointsY = new int[]{120, 215, 100, 120, 120,215};

    Scalar scalarLow, scalarHigh;

    @Override
    public Mat processFrame(Mat input) {
//        EOCVWebcam.pipelineName = "Color Edge Detection Bounded";
        // color map below
        // https://i.stack.imgur.com/gyuw4.png
        if (alliance == Alliance.RED){
            scalarLow = new Scalar(126, 131,100);
            scalarHigh = new Scalar(221,255,184);
        }else if (alliance == Alliance.BLUE){
            //todo change to blue
            scalarLow = new Scalar(0, 65,124);
            scalarHigh = new Scalar(96,126,190);
        }
        Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(input, new Point(pointsX[4], pointsY[4]), new Point(pointsX[5], pointsY[5]), new Scalar(0, 255, 0), 1);
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
            if (((centers[i].x > pointsX[0] && centers[i].x < pointsX[1]) && (centers[i].y > pointsY[0] && centers[i].y < pointsY[1])) ||
                    ((centers[i].x > pointsX[2] && centers[i].x < pointsX[3]) && (centers[i].y > pointsY[2] && centers[i].y < pointsY[3])) ||
                    ((centers[i].x > pointsX[4] && centers[i].x < pointsX[5]) && (centers[i].y > pointsY[4] && centers[i].y < pointsY[5])))
            {
                Imgproc.drawContours(input, contoursPolyList, i, c);
                Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), c, 1);
                double centerX = boundRect[i].tl().x + ((boundRect[i].br().x - boundRect[i].tl().x) / 2);
                double centerY = boundRect[i].br().y + ((boundRect[i].tl().y - boundRect[i].br().y) / 2);
                Imgproc.line(input, new Point(centerX - 5, centerY), new Point(centerX + 5, centerY), new Scalar(0, 255, 200));
                Imgproc.line(input, new Point(centerX, centerY - 5), new Point(centerX, centerY + 5), new Scalar(0, 255, 200));
                if (boundRect[i].height > boundRect[highIndex].height && boundRect[i].width > boundRect[highIndex].width)//get largest rectangle
                    highIndex = i;
            }
        }
        // check which side it is actually on using centers
        if (highIndex != 0) {
            if (centers[highIndex].x > pointsX[0] && centers[highIndex].x < pointsX[1]) {
                if (centers[highIndex].y > pointsY[0] && centers[highIndex].y < pointsY[1]) {
                    Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
                    Imgproc.putText(input, "left", new Point(pointsX[0], pointsY[0]), 0, 1, new Scalar(0, 255, 0));
                    autonomousRandom = AutoRandom.left;
                }
            }
            if (centers[highIndex].x > pointsX[2] && centers[highIndex].x < pointsX[3]) {
                if (centers[highIndex].y > pointsY[2] && centers[highIndex].y < pointsY[3]) {
                    Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
                    Imgproc.putText(input, "center", new Point(pointsX[2], pointsY[2]), 0, 1, new Scalar(0, 255, 0));
                    autonomousRandom = AutoRandom.mid;
                }
            }
            if (centers[highIndex].x > pointsX[4] && centers[highIndex].x < pointsX[5]) {
                if (centers[highIndex].y > pointsY[4] && centers[highIndex].y < pointsY[5]) {
                    Imgproc.rectangle(input, new Point(pointsX[4], pointsY[4]), new Point(pointsX[5], pointsY[5]), new Scalar(0, 255, 0), 1);
                    Imgproc.putText(input, "right", new Point(pointsX[4], pointsY[4]), 0, 1, new Scalar(0, 255, 0));
                    autonomousRandom = AutoRandom.right;
                }
            }
        }

        return input;
    }
}

