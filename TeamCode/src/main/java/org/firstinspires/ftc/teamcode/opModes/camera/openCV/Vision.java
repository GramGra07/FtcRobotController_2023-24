package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Vision {

    Mat needle_img = null;
    int needle_w = 0;
    int needle_h = 0;
    int method = 0;

    public Vision(String needle_img_path){
        if (needle_img_path != null && !needle_img_path.isEmpty()){
            this.needle_img = Imgcodecs.imread(needle_img_path, Imgcodecs.IMREAD_UNCHANGED);
            this.needle_w = this.needle_img.width();
            this.needle_h = this.needle_img.height();
        }
        this.method = method;
    }
    Mat hierarchy = new Mat();
    Mat mask = new Mat();

    public Mat find(Mat haystack_img, double threshold, int max_results){
        Mat result = new Mat();
        Imgproc.matchTemplate(haystack_img, this.needle_img, result, this.method);
        Core.MinMaxLocResult mmr = Core.minMaxLoc(result);
        double maxVal = mmr.maxVal;
        if (maxVal >= threshold){
            List<Mat> channels = new ArrayList<>();
            Core.split(result, channels);
            Imgproc.threshold(channels.get(0), mask, threshold, 255, Imgproc.THRESH_BINARY);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            int results = Math.min(max_results, contours.size());
            Mat locations = new Mat(results, 4, CvType.CV_8UC3);
            for (int i = 0; i < results; i++){
                Mat contour = contours.get(i);
                double[] data = new double[4];
                data[0] = contour.get(0, 0)[0];
                data[1] = contour.get(0, 0)[1];
                data[2] = this.needle_w;
                data[3] = this.needle_h;
                locations.put(i, 0, data);
            }
            return locations;
        }
        return new Mat();
    }
    public List<Point> getClickPoints(List<Rect> rectangles) {
        List<Point> points = new ArrayList<>();
        for (Rect rect : rectangles) {
            int center_x = rect.x + rect.width/2;
            int center_y = rect.y + rect.height/2;
            points.add(new Point(center_x, center_y));
        }
        return points;
    }

    public Mat drawRectangles(Mat haystackImg, List<Rect> rectangles) {
        Scalar lineColor = new Scalar(0, 255, 0);
        int lineType = Imgproc.LINE_4;
        for (Rect rect : rectangles) {
            Point topLeft = new Point(rect.x, rect.y);
            Point bottomRight = new Point(rect.x + rect.width, rect.y + rect.height);
            Imgproc.rectangle(haystackImg, topLeft, bottomRight, lineColor, lineType);
        }
        return haystackImg;
    }

    public Mat drawCrosshairs(Mat haystackImg, List<Point> points) {
        Scalar markerColor = new Scalar(255, 0, 255);
        int markerType = Imgproc.MARKER_CROSS;
        for (Point point : points) {
            Imgproc.drawMarker(haystackImg, point, markerColor, markerType);
        }
        return haystackImg;
    }
}
