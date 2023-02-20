package org.firstinspires.ftc.teamcode.externalHardware;

import static android.os.SystemClock.sleep;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class autoHardware extends HardwareConfig {
    public double ovrPower = 0.5;
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }

    public void initAuto(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        init(ahwMap);
        //not same
        myOpMode.telemetry.addData("Starting at", "%7d :%7d",
                motorBackRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition());
        closeClaw();
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        runVu(6, false);
        myOpMode.telemetry.update();
        closeClaw();
        tmServo.setPosition(setServo(tmPose));
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

    public void correctToCones() {
        correctByColor();
        correctByTouch();
    }

    public void correctByColor() {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        getAllColorR();
        getAllColorL();
        NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
        Color.colorToHSV(colorsR.toColor(), hsvValues);
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        float redValR = colorsR.red;//the red value in rgb
        float greenValR = colorsR.green;//the green value in rgb
        float blueValR = colorsR.blue;//the blue value in rgb
        float redValL = colors.red;//the red value in rgb
        float greenValL = colors.green;//the green value in rgb
        float blueValL = colors.blue;//the blue value in rgb
        //right
        double redTargetRR = 0.003;//the red value in rgb
        double redTargetGR = 0.004;//the green value in rgb
        double redTargetBR = 0.003;//the blue value in rgb
        //left
        double redTargetRL = 0.003;//the red value in rgb
        double redTargetGL = 0.003;//the green value in rgb
        double redTargetBL = 0.002;//the blue value in rgb
        //right
        double blueTargetRR = 0.002;//the red value in rgb
        double blueTargetGR = 0.004;//the green value in rgb
        double blueTargetBR = 0.005;//the blue value in rgb
        //left
        double blueTargetRL = 0.001;//the red value in rgb
        double blueTargetGL = 0.003;//the green value in rgb
        double blueTargetBL = 0.0038;//the blue value in rgb
        double range = 0.0005;
        //left
        while (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
                || colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
            if ((colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                    || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range))) {
                getAllColorR();
                sideWaysEncoderDrive(1, 0.25, 0.4);//go left
                //right side has seen red or blue
            }
            if (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                    || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)) {
                getAllColorL();
                sideWaysEncoderDrive(1, -0.25, 0.4);//go right
            }
            if (!colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                    || !colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
                    || !colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                    || !colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
                break;
            }
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }


    public void correctByTouch() {
        boolean pressed = touchSensor.isPressed();
        boolean pressedL = touchSensorL.isPressed();
        while (!pressed && !pressedL) {
            pressed = touchSensor.isPressed();
            pressedL = touchSensorL.isPressed();
            if (pressed && pressedL) {
                break;
            }
            encoderDrive(1, -6, -6, 1);
        }
    }

    public void doSetup() {
        runVu(6, true);
        if (spot == 0) {
            spot = 2;
        }
        reportSpot(spot);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }

    public void reportSpot(int spot) {
        if (spot == 1) {
            green1.setState(true);
            red1.setState(false);
        } else if (spot == 2) {
            green1.setState(true);
            red1.setState(false);
            green2.setState(true);
            red2.setState(false);
        } else if (spot == 3) {
            green1.setState(true);
            red1.setState(false);
            green2.setState(true);
            red2.setState(false);
            green3.setState(true);
            red3.setState(false);
        }
    }

    public void score1() {
        doSetup();
        //branch 1 get to spot
        encoderDrive(1, -1, -1, 0.5);
        simplerGoSpot(ovrCurrX, ovrCurrY, 1, 2, ovrPower, false, 0, false
                , false, 0, 1, 4);
        setOvr(1, 2);
        double targetX = -0.02;
        double targetY2 = 3.55;// at pole
        sleep(50);
        resetEncoders();
        simplerGoSpot(1, 3, targetX, targetY2, ovrPower, true, topPoleVal,
                false, false, 0, 2, 4);
        turn(80);
        resetEncoders();
        double fw = -2;
        encoderDrive(1, fw, fw, 1);
        setOvr(targetX, targetY2);
        sleep(500);
        openClaw();
        sleep(200);
        closeClaw();
        sleep(200);
        encoderDrive(1, -fw, -fw, 1);
    }

    public void parkFrom2_() {
        double stackDist = -15;
        sideWaysEncoderDrive(ovrPower, -6, 2);
        yArmEncoder(0, 1, 2, true);
        sleep(50);
        if (spot == 3) {
            encoderDrive(1, stackDist, stackDist, 3);//opposite of 3 lines higher
            //3,3
        }
        //should already be here at spot 2
        if (spot == 2) {
            //2,3
        }
        if (spot == 1) {
            encoderDrive(1, -stackDist, -stackDist, 3);
            //1,3
        }
    }

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
            yArmEncoder(pose, 1, 2, isUp);
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
            yArmEncoder(pose, 1, 2, isUp);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        myOpMode.telemetry.update();
    }

}
