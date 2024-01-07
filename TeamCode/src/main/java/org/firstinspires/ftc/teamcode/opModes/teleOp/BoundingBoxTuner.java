package org.firstinspires.ftc.teamcode.opModes.teleOp;

import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam1_N;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.BoundingPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(group = "b")
public class BoundingBoxTuner extends LinearOpMode {
    private OpenCvWebcam webcam; // the webcam public we are using
    public static  int[] xPoints;
    public static int[] yPoints;
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
        webcam.setPipeline(new BoundingPipeline()); // set the webcam pipeline to the OBJDetect2 pipeline
        FtcDashboard.getInstance().startCameraStream(webcam, 0); // start the camera stream on FTC Dash
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
        waitForStart();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        if (opModeIsActive()){
            telemetry.addData("Press triangle if tuning blue","press x if tuning red");
            telemetry.update();
            while (!gamepad1.triangle || gamepad1.cross){
                if (isStopRequested()) {
                    return;
                }
            }
            if (gamepad1.triangle){
                BoundingPipeline.scalarLow = new Scalar(0, 0, 130);
                BoundingPipeline.scalarHigh = new Scalar(255, 255, 255);
            }
            if (gamepad1.cross){
                BoundingPipeline.scalarLow = new Scalar(0, 147, 0);
                BoundingPipeline.scalarHigh = new Scalar(255, 255, 255);
            }
            telemetry.clearAll();
            telemetry.addData("Tuning middle first, please place prop on the middle spike mark","Press dpad up to take");
            telemetry.update();
            while (!gamepad1.dpad_up){
                if (isStopRequested()){
                    return;
                }
            }
            telemetry.clearAll();
            telemetry.addData("Tuning right next, please place on right mark","Press dpad down to take picture");
            telemetry.update();
            while (!gamepad1.dpad_down){
                if (isStopRequested()){
                    return;
                }
            }
            xPoints = new int[] {(int) BoundingPipeline.boundingBoxCenter[0], (int) BoundingPipeline.boundingBoxCenter[1],(int) BoundingPipeline.boundingBoxRight[0], (int) BoundingPipeline.boundingBoxRight[1]};
            yPoints = new int[] {(int) BoundingPipeline.boundingBoxCenter[2], (int) BoundingPipeline.boundingBoxCenter[3],(int) BoundingPipeline.boundingBoxRight[2], (int) BoundingPipeline.boundingBoxRight[3]};
            telemetry.addData("public static int[] pointsX = new int[]{"+xPoints[0]+","+xPoints[1]+","+xPoints[2]+","+xPoints[3]+","+"};","\npublic static int[] pointsY = new int[]{"+yPoints[0]+","+yPoints[1]+","+yPoints[2]+","+yPoints[3]+","+"};");
            telemetry.update();
            while(!isStopRequested()){
                if (isStopRequested()){
                    return;
                }
            }
        }

    }
}
