package org.firstinspires.ftc.teamcode.externalHardware;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.externalHardware.MathFunctions.setOvr;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class autoHardware extends HardwareConfig {//auto version of hardware config
    public double ovrPower = 0.5;//sets power
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }

    public void initAuto(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        init(ahwMap);//initializes with hardware config
        //not same
        myOpMode.telemetry.addData("Starting at", "%7d :%7d",
                motorBackRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition());
        // ONLY use for vuforia
        //initVuforia();
        //initTfod();
        //if (tfod != null) {
        //    tfod.activate();
        //    tfod.setZoom(1.0, 16.0 / 9.0);
        //}
        //runVu(6, false);
        myOpMode.telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        sleep(300);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        myOpMode.waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }

    public void correctByImu(float currentAngle, int targetAngle) {
        int angle = (int) (targetAngle - currentAngle);
        turn(angle);
    }

    //public void correctByColor() {
    //    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    //    getAllColorR();
    //    getAllColorL();
    //    NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
    //    Color.colorToHSV(colorsR.toColor(), hsvValues);
    //    NormalizedRGBA colors = colorSensorL.getNormalizedColors();
    //    Color.colorToHSV(colors.toColor(), hsvValues);
    //    float redValR = colorsR.red;//the red value in rgb
    //    float greenValR = colorsR.green;//the green value in rgb
    //    float blueValR = colorsR.blue;//the blue value in rgb
    //    float redValL = colors.red;//the red value in rgb
    //    float greenValL = colors.green;//the green value in rgb
    //    float blueValL = colors.blue;//the blue value in rgb
    //    //right
    //    double redTargetRR = 0.003;//the red value in rgb
    //    double redTargetGR = 0.004;//the green value in rgb
    //    double redTargetBR = 0.003;//the blue value in rgb
    //    //left
    //    double redTargetRL = 0.003;//the red value in rgb
    //    double redTargetGL = 0.003;//the green value in rgb
    //    double redTargetBL = 0.002;//the blue value in rgb
    //    //right
    //    double blueTargetRR = 0.002;//the red value in rgb
    //    double blueTargetGR = 0.004;//the green value in rgb
    //    double blueTargetBR = 0.005;//the blue value in rgb
    //    //left
    //    double blueTargetRL = 0.001;//the red value in rgb
    //    double blueTargetGL = 0.003;//the green value in rgb
    //    double blueTargetBL = 0.0038;//the blue value in rgb
    //    double range = 0.0005;
    //    //left
    //    while (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
    //            || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
    //            || colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
    //            || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
    //        if ((colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
    //                || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range))) {
    //            getAllColorR();
    //            sideWaysEncoderDrive(1, 0.25, 0.4);//go left
    //            //right side has seen red or blue
    //        }
    //        if (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
    //                || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)) {
    //            getAllColorL();
    //            sideWaysEncoderDrive(1, -0.25, 0.4);//go right
    //        }
    //        if (!colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
    //                || !colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
    //                || !colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
    //                || !colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
    //            break;
    //        }
    //    }
    //    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    //}


    //public void correctByTouch() {
    //    boolean pressed = touchSensor.isPressed();
    //    boolean pressedL = touchSensorL.isPressed();
    //    while (!pressed && !pressedL) {
    //        pressed = touchSensor.isPressed();
    //        pressedL = touchSensorL.isPressed();
    //        if (pressed && pressedL) {
    //            break;
    //        }
    //        encoderDrive(1, -6, -6, 1);
    //    }
    //}
    public void simpleGoSpotRight(double currX, double currY, double targetX, double targetY, double power,
                                  boolean combo, int pose, boolean isUp, boolean endTurn, int turn, double timeOutX,
                                  double timeOutY, boolean prioritizeY) {
        double XMULT = 9.0;
        double YMULT = 20.0;
        double sidewaysInches = (targetY - currY) * XMULT * -1;
        double fwdInches = (targetX - currX) * YMULT;
        if (prioritizeY) {
            sideWaysEncoderDrive(power, sidewaysInches, timeOutY);
            sleep(100);
        }
        if (!combo) {
            encoderDrive(power, fwdInches, fwdInches, timeOutX);
        } else {
            encoderDrive(power, fwdInches, fwdInches, timeOutX);
            // arm encoder
        }
        if (!prioritizeY) {
            sleep(100);
            sideWaysEncoderDrive(power, sidewaysInches, timeOutY);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        myOpMode.telemetry.update();
    }

    public void simplerGoSpot(double currX, double currY, double targetX, double targetY, double power, boolean combo, int pose
            , boolean isUp, boolean endTurn, int turn, int timeOutX, int timeOutY) {
        double sidewaysInches = (targetX - currX) * xMult;
        double fwdInches = (targetY - currY) * yMult;
        myOpMode.telemetry.addData("fwdInches", fwdInches);
        myOpMode.telemetry.addData("sidewaysInches", sidewaysInches);
        myOpMode.telemetry.update();
        sideWaysEncoderDrive(power, sidewaysInches, timeOutX);
        if (!combo) {
            encoderDrive(power, -fwdInches, -fwdInches, timeOutY);
        } else {
            encoderDrive(power, -fwdInches, -fwdInches, timeOutY);
            // arm encoder
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        myOpMode.telemetry.update();
    }

}
