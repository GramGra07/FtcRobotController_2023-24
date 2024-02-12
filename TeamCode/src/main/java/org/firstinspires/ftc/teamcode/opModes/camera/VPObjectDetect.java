package org.firstinspires.ftc.teamcode.opModes.camera;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.lights;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.autonomousRandom;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class VPObjectDetect implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    Mat ycrcbMat = new Mat();
    Mat right = new Mat();
    Mat middle = new Mat();
    public Scalar c = new Scalar(255, 0, 0);
    public Alliance alliance;

    public VPObjectDetect(Alliance color) {
        this.alliance = color;
    }

    int current = 0;

    public static int[] pointsX = new int[]{570, 680, 120, 230};
    public static int[] pointsY = new int[]{70, 170, 50, 150};

    public Scalar scalarLow, scalarHigh;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (alliance == Alliance.RED) {
            scalarLow = new Scalar(0, 140, 0);
            scalarHigh = new Scalar(255, 255, 255);
        } else if (alliance == Alliance.BLUE) {
            scalarLow = new Scalar(0, 0, 130);
            scalarHigh = new Scalar(255, 255, 255);
        }
        Imgproc.rectangle(frame, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(frame, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
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
                    Imgproc.rectangle(frame, new Point(pointsX[2], pointsY[2]), new Point(pointsX[3], pointsY[3]), new Scalar(0, 255, 0), 1);
                    Imgproc.putText(frame, "right", new Point(frame.width() / 2, frame.height() / 2), 0, 5, new Scalar(0, 255, 0));
                    autonomousRandom = AutoRandom.right;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    current = 1;
//                    shiftOffset = rightShift;
                }
            }
        }
        if (current != 1) {
            if (middleMean[0] > scalarLow.val[0] && middleMean[0] < scalarHigh.val[0]) {
                if (middleMean[1] > scalarLow.val[1] && middleMean[1] < scalarHigh.val[1]) {
                    if (middleMean[2] > scalarLow.val[2] && middleMean[2] < scalarHigh.val[2]) {
                        Imgproc.rectangle(frame, new Point(pointsX[0], pointsY[0]), new Point(pointsX[1], pointsY[1]), new Scalar(0, 255, 0), 1);
                        Imgproc.putText(frame, "middle", new Point(frame.width() / 2, frame.height() / 2), 0, 5, new Scalar(0, 255, 0));
                        autonomousRandom = AutoRandom.mid;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                        current = 1;
//                        shiftOffset = 0;
                    }
                }
            }
        }
        if (current == 0) {
            Imgproc.putText(frame, "left", new Point(frame.width() / 2, frame.height() / 2), 0, 5, new Scalar(0, 255, 0));
            autonomousRandom = AutoRandom.left;
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
//            shiftOffset = -leftShift;
        }
        current = 0;
        ycrcbMat.release();
        right.release();
        middle.release();
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
