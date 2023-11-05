package org.firstinspires.ftc.teamcode.eocvSim;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SimpleThresholdPipeline extends OpenCvPipeline {
    public static int lower1 = 0;
    public static int lower2 = 147;
    public static int lower3 = 0;
    public static int higher1 = 255;
    public static int higher2 = 255;
    public static int higher3 = 255;
    public static Scalar lower = new Scalar(lower1, lower2, lower3);
    public static Scalar upper = new Scalar(higher1, higher2, higher3);
    private final Mat ycrcbMat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
        return maskedInputMat;
    }
}