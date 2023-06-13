package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamExample extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new EdgeDetectionPipeline());//!can switch pipelines here

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
            if (gamepad1.a) {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                telemetry.addData("Webcam", "Stopping");
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
    class EdgeDetectionPipeline extends OpenCvPipeline {
        Mat gray = new Mat();
        Mat edges = new Mat();
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(gray, edges, 50, 100);
            return edges;
        }
        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}
//// Process frame if available
//            if (frame != null) {
//                    // Perform edge detection
//                    Mat edges = performEdgeDetection(frame);
//
//                    // Show the edges
//                    showImage("Edges", edges);
//
//                    // Release the frame and edges to free up memory
//                    frame.release();
//                    edges.release();
//                    }
//
//                    idle();
//                    }
//
//                    // Disable and release the camera view
//                    if (cameraView != null) {
//                    cameraView.disableView();
//                    }
//                    }
//
//private Mat performEdgeDetection(Mat frame) {
//        // Convert the frame to grayscale
//        Mat grayImage = new Mat();
//        Imgproc.cvtColor(frame, grayImage, Imgproc.COLOR_RGBA2GRAY);
//
//        // Apply edge detection algorithm (Canny edge detector)
//        Mat edges = new Mat();
//        Imgproc.Canny(grayImage, edges, threshold1, threshold2);
//
//        // Release the grayscale image
//        grayImage.release();
//
//        // Return the edges frame
//        return edges;
//        }
//
//private void showImage(String windowName, Mat image) {
//        // Create a Bitmap to hold the image
//        Bitmap bitmap = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
//
//        // Convert the Mat image to a Bitmap
//        Utils.matToBitmap(image, bitmap);
//
//        // Show the Bitmap in FTC telemetry
//        telemetry.addData(windowName, bitmap);
//        telemetry.update();
//        }
//        }