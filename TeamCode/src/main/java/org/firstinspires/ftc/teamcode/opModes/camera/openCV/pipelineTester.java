package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
//@Disabled
public class pipelineTester extends LinearOpMode {
    OpenCvWebcam webcam;
    public static int x = 0;
    public static double left = 0;
    public static double right = 0;
    public static double top = 0;
    public static double bottom = 0;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
        webcam.setPipeline(new OpenCVpipelines.RecognizeObject("red", "prop"));//!can switch pipelines here
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        // OpenCVpipelines.WhiteDotDetection()
        // OpenCVpipelines.ColorEdgeDetectionBounded("yellow")
        // RecognizeObject("red", "cone")
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
            telemetry.addData("Pipeline", pipelineName);
            telemetry.addData("whiteDots", whiteDots);
            telemetry.addData("blackDots", blackDots);
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            telemetry.addData("Top", top);
            telemetry.addData("Bottom", bottom);
            telemetry.update();
            if (gamepad1.a) {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
            sleep(100);
        }
    }
}