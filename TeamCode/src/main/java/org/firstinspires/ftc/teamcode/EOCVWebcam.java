package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Vision.cameraWidth;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.camera.openCV.OpenCVpipelines;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class EOCVWebcam {
    //webcam
    public final static String cam1_N = "Webcam 1";
    public final static String cam2_N = "Webcam 2";
    public static String pipelineName = "";
    public static int whiteDots = 0, blackDots = 0;
    public static double blackDotCenterX = 0, blackDotCenterY = 0;
    public static double centerX = 0, centerY;
    public static void initEOCV(HardwareMap hardwareMap, OpenCvWebcam webcam){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cam1_N), cameraMonitorViewId);
        webcam.setPipeline(new OpenCVpipelines.RecognizeObject("red", "prop"));//!can switch pipelines here
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam1 = webcam;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }
    public static void closeCamera(OpenCvWebcam webcam){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
    public static void extrapolateCenter(){
        double cameraThird = 120;

        Sensors.ledIND(HardwareConfig.green4,HardwareConfig.red4,false);
        if (EOCVWebcam.centerX < cameraThird){
            autoHardware.autonomousRandom = AutoRandom.left;
            Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
            Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,false);
            Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,false);
        }else if (EOCVWebcam.centerX > cameraThird && EOCVWebcam.centerX < cameraThird*2){
            autoHardware.autonomousRandom = AutoRandom.mid;
            Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
            Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,true);
            Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,false);
        }else if (EOCVWebcam.centerX > cameraThird*2){
            autoHardware.autonomousRandom = AutoRandom.right;
            Sensors.ledIND(HardwareConfig.green1,HardwareConfig.red1,true);
            Sensors.ledIND(HardwareConfig.green2,HardwareConfig.red2,true);
            Sensors.ledIND(HardwareConfig.green3,HardwareConfig.red3,true);
        }
    }
}
