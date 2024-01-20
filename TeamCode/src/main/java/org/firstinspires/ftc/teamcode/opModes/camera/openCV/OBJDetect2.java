package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autonomousRandom;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@Config
public class OBJDetect2 extends OpenCvPipeline {
    Mat ycrcbMat = new Mat();
    Mat right = new Mat();
    Mat middle = new Mat();
    public Scalar c = new Scalar(255, 0, 0);
    public Alliance alliance;

    public OBJDetect2(Alliance color) {
        this.alliance = color;
    }

    int current = 0;

    public static int[] pointsX = new int[]{560, 560 + 120, 100, 250};
    public static int[] pointsY = new int[]{70, 200, 40, 190};

    public Scalar scalarLow, scalarHigh;

    @Override
    public Mat processFrame(Mat input) {
        if (alliance == Alliance.RED) {
            scalarLow = new Scalar(0, 147, 0);
            scalarHigh = new Scalar(255, 255, 255);
        } else if (alliance == Alliance.BLUE) {
            scalarLow = new Scalar(0, 0, 130);
            scalarHigh = new Scalar(255, 255, 255);
        }
        Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        middle = ycrcbMat.submat(new Rect(pointsX[0], pointsY[0], pointsX[1] - pointsX[0], pointsY[1] - pointsY[0]));
        right = ycrcbMat.submat(new Rect(pointsX[2], pointsY[2], pointsX[3] - pointsX[2], pointsY[3] - pointsY[2]));
        Core.mean(right);
        Core.mean(middle);
        double[] rightMean = Core.mean(right).val;
        double[] middleMean = Core.mean(middle).val;
        // check if it is within the scalar low and high
        if (rightMean[0] > scalarLow.val[0] && rightMean[0] < scalarHigh.val[0]) {
            if (rightMean[1] > scalarLow.val[1] && rightMean[1] < scalarHigh.val[1]) {
                if (rightMean[2] > scalarLow.val[2] && rightMean[2] < scalarHigh.val[2]) {
                    Imgproc.rectangle(input, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
                    Imgproc.putText(input, "right", new Point(input.width() / 2, input.height() / 2), 0, 5, new Scalar(0, 255, 0));
                    autonomousRandom = AutoRandom.right;
                    Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                    Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                    Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
                    Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                    current = 1;
                }
            }
        }
        if (current != 1) {
            if (middleMean[0] > scalarLow.val[0] && middleMean[0] < scalarHigh.val[0]) {
                if (middleMean[1] > scalarLow.val[1] && middleMean[1] < scalarHigh.val[1]) {
                    if (middleMean[2] > scalarLow.val[2] && middleMean[2] < scalarHigh.val[2]) {
                        Imgproc.rectangle(input, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
                        Imgproc.putText(input, "middle", new Point(input.width() / 2, input.height() / 2), 0, 5, new Scalar(0, 255, 0));
                        autonomousRandom = AutoRandom.mid;
                        Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                        Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                        Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
                        Sensors.ledIND(HardwareConfig.green4, HardwareConfig.red4, true);
                        current = 1;
                    }
                }
            }
        }
        if (current == 0) {
            Imgproc.putText(input, "left", new Point(input.width() / 2, input.height() / 2), 0, 5, new Scalar(0, 255, 0));
            autonomousRandom = AutoRandom.left;
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
            Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
            Sensors.ledIND(HardwareConfig.green4, HardwareConfig.red4, false);
        }
        current = 0;
        ycrcbMat.release();
        right.release();
        middle.release();
        return input;
    }
}