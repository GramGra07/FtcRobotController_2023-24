//import
package org.firstinspires.ftc.teamcode.opModes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.blink;
import org.firstinspires.ftc.teamcode.opModes.configVars.varConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.DistanceStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class HardwareConfig {//this is an external opMode that can have public variables used by everything
    boolean reverse = false;
    //my personal key
    public static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    public TFObjectDetector tfod;
    //other variables
    public String statusVal = "OFFLINE";
    //servo variables
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;// = 100/180
    public static final ElapsedTime timer = new ElapsedTime();

    //rumble
    public Gamepad.RumbleEffect customRumbleEffect, customRumbleEffect1;//declaring the customRumbleEffect variable
    public final double endgame = 120, end = 150;//declaring the endgame variable
    public boolean isEndgame = false, isEnd = false;//declaring the isEndgame variable

    //motors
    public static DcMotor motorFrontLeft = null, motorBackLeft = null, motorFrontRight = null, motorBackRight = null;
    //encoders
    public static DcMotor enc1 = null;

    //lights
    public static RevBlinkinLedDriver lights;
    //slow mode
    public int slowMult = varConfig.slowMult, slowPower;
    public boolean slowModeIsOn = false, reversed;
    //driving
    public double xControl, yControl, frontRightPower, frontLeftPower, backRightPower, backLeftPower;
    //field centric
    double gamepadX, gamepadY, gamepadHypot, controllerAngle, robotDegree, movementDegree;
    //imu
    //public static BNO055IMU imu;
    //public static Orientation angles;     //imu uses these to find angles and classify them
    //public Acceleration gravity;    //Imu uses to get acceleration
    //public double primaryHeading, primaryRoll, primaryPitch;
    //public double heading, roll, pitch;
    //maintenance mode
    public int delay = varConfig.delay;
    public boolean isSolid = false;
    public static String LEDcolor;
    //encoder vals


    //rev potentiometer
    public static final double POTENTIOMETER_MAX = 270;
    public static final double POTENTIOMETER_MIN = 0;
    public AnalogInput potentiometer;

    // rev magnetic limit switch
    public DigitalChannel limitSwitch;
    public boolean limitSwitchState = false;

    //external
    HardwareMap hardwareMap = null;

    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public HardwareConfig(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //voltage

    public VoltageSensor vSensor;
    public boolean lowVoltage = false;
    public final double minimumVoltage = 11.5;
    public double currentVoltage;
    public boolean once = false;

    //switchable profile variables
    public final String[] driverControls = {"Chase", "Camden", "Graden", "Kian", "Child"}, otherControls = driverControls;
    public final int baseDriver = 0, baseOther = 1;//list integer of base driver and other controls
    public int dIndex = baseDriver, oIndex = baseOther;//list integer of driver and other controls
    public String currDriver = driverControls[dIndex], currOther = otherControls[oIndex];//list string of driver and other controls
    boolean fieldCentric;
    public boolean optionsHigh1 = false, shareHigh1 = false, optionsHigh2 = false, shareHigh2 = false;
    public boolean dDownHigh = false;

    //webcam
    public final static String cam1_N = "Webcam 1";
    public final static String cam2_N = "Webcam 2";
    public static String pipelineName = "";
    public final static double minConfidence = varConfig.minConfidence;
    public static int whiteDots = 0;
    public static int blackDots = 0;
    //rr
    public SampleMecanumDrive drive = null;
    public static double thisDist = 0;

    //temp vars
    public final int leftR = 4;
    public final int midR = 5;
    public final int rightR = 6;
    public final int leftB = 7;
    public final int midB=8;
    public final int rightB=9;
    public final int aprilTagB = 10;
    public final int aprilTagR = 11;


    //file write
    public final String file = String.format("%s/FIRST/matchlogs/log.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    FileWriter fileWriter;

    public void setUpFile() {
        File myObj = new File(file);
        try {
            myObj.delete();
            myObj.createNewFile();
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            fileWriter.write("");
            fileWriter.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void writeToFile(int x, int y) {
        //  terminal
        //  clear
        //  adb shell
        //  cat /storage/emulated/0/FIRST/matchlogs/log.txt
        //  copy everything
        //  paste into file.txt
        //  run testGraphing in pycharm or other python IDE
        //  look at results in graph.png
        try {
            fileWriter = new FileWriter(file, true);
            fileWriter.append(String.valueOf(x)).append(" ").append(String.valueOf(y)).append(" \n");
            fileWriter.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public final String currentVersion = "3.0.1";

    //init
    public void init(HardwareMap ahwMap) {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        // Telemetry telemetry = myOpMode.telemetry;
        thisDist = 0;
        setUpFile();
        updateStatus("Initializing");
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        ElapsedTime timer = new ElapsedTime();//declaring the runtime variable
        vSensor = ahwMap.voltageSensor.get("Expansion Hub 2");//getting the voltage sensor
        getBatteryVoltage();
        //imu
        //if (!auto) {
        //    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //    parameters.loggingEnabled = true;
        //    parameters.loggingTag = "IMU";
        //    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //    imu = ahwMap.get(BNO055IMU.class, "imu");
        //    imu.initialize(parameters);
        //    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        //    gravity = imu.getGravity();
        //    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //}

        lights = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        // rev potentiometer //analog
        potentiometer = ahwMap.get(AnalogInput.class, "potent");
        //magnetic limit switch //digital is pressed
        limitSwitch = ahwMap.get(DigitalChannel.class, "limitSwitch");
        // Declare our motors
        motorFrontLeft = ahwMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = ahwMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = ahwMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = ahwMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        //encoders
        enc1 = ahwMap.get(DcMotor.class, "enc1");
        enc1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //reversals
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //set all to brake when set 0 power
        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        timer.reset();//resetting the runtime variable
        //LED
        blink.setLights(null, true);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color", LEDcolor);
        telemetry.addData("Version", currentVersion);
        telemetry.addData("Voltage", "%.2f", currentVoltage);
        if (lowVoltage) {
            telemetry.addData("lowBattery", "true");
        }
        telemetry.update();

    }

    //code to run all drive functions
    public void doBulk() {
        once();//runs once
        bindDriverButtons();
        bindOtherButtons();
        switchProfile();
        drive(fieldCentric);
        power();//sets power to power variables
        buildTelemetry();//makes telemetry
    }

    public void once() {
        if (!once) {
            Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
            // Telemetry telemetry = myOpMode.telemetry;
            telemetry.clearAll();
            updateStatus("Running");
            //findOrientationOffset();
            once = true;
        }
    }

    public void drive(boolean fieldCentric) {
        if (fieldCentric) {
            gamepadX = myOpMode.gamepad1.left_stick_x;//get the x val of left stick and store
            //telemetry.addData("gamepadX", gamepadX);//tell us what gamepadX is
            gamepadY = -myOpMode.gamepad1.left_stick_y;//get the y val of left stick and store
            //telemetry.addData("gamepadY", gamepadY);//tell us what gamepadY is
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);//get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            //telemetry.addData("gamepadHypot", gamepadHypot);//tell us what gamepadHypot is
            controllerAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX));//Get the angle of the controller stick using arc tangent
            //telemetry.addData("controllerAngle", controllerAngle);//tell us what controllerAngle is
            //might need to change based on corrected heading
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//get and initialize the IMU
            //robotDegree = angles.firstAngle;//store robot angle in robotDegree
            robotDegree = Math.toDegrees(drive.getPoseEstimate().getHeading());
            //telemetry.addData("robotDegree", robotDegree);//tell us what robotDegree is
            movementDegree = (controllerAngle - robotDegree);//get the movement degree based on the controller vs robot angle
            //telemetry.addData("movementDegree", movementDegree);//tell us what movementDegree is
            xControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;//get the x value of the movement
            //telemetry.addData("xControl", xControl);//tell us what xControl is
            yControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;//get the y value of the movement
            //telemetry.addData("yControl", yControl);//tell us what yControl is
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower;
            backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower;
            frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower;
            backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower;
        } else {
            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
            reversed = reverse;
            yControl = -myOpMode.gamepad1.left_stick_y;
            xControl = myOpMode.gamepad1.left_stick_x;
            if (reverse) {
                yControl = -yControl;
                xControl = -xControl;
            }
            double turn = -myOpMode.gamepad1.right_stick_x;
            if (slowModeIsOn) slowPower = slowMult;
            frontRightPower = (yControl - xControl + turn) / slowPower;
            backRightPower = (yControl + xControl + turn) / slowPower;
            frontLeftPower = (yControl + xControl - turn) / slowPower;
            backLeftPower = (yControl - xControl - turn) / slowPower;
        }
        drive.update();
        updateDistTraveled(PoseStorage.currentPose, drive.getPoseEstimate());
        writeToFile((int) drive.getPoseEstimate().getX(), (int) drive.getPoseEstimate().getY());
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void updateDistTraveled(Pose2d before, Pose2d after) {
        double x = after.getX() - before.getX();
        double y = after.getY() - before.getY();
        double dist = Math.sqrt((x * x) + (y * y));
        thisDist += dist;
        DistanceStorage.totalDist += dist;
    }

    public void switchProfile() {
        //driver
        if (myOpMode.gamepad1.options && !optionsHigh1) {
            if (dIndex == driverControls.length - 1) {
                dIndex = 0;
            } else {
                dIndex++;
            }
            currDriver = driverControls[dIndex];
        }
        optionsHigh1 = myOpMode.gamepad1.options;
        if (myOpMode.gamepad1.share && !shareHigh1) {
            if (dIndex == 0) {
                dIndex = driverControls.length - 1;
            } else {
                dIndex--;
            }
            currDriver = driverControls[dIndex];
        }
        shareHigh1 = myOpMode.gamepad1.share;
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2) {
            if (oIndex == otherControls.length - 1) {
                oIndex = 0;
            } else {
                oIndex++;
            }
            currOther = otherControls[oIndex];
        }
        optionsHigh2 = myOpMode.gamepad2.options;
        if (myOpMode.gamepad2.share && !shareHigh2) {
            if (oIndex == 0) {
                oIndex = otherControls.length - 1;
            } else {
                oIndex--;
            }
            currOther = otherControls[oIndex];
        }
        shareHigh2 = myOpMode.gamepad2.share;
    }

    public void bindDriverButtons() {
        //"Chase", "Camden","Graden","Kian","Child"
        if (currDriver == driverControls[0]) {//Chase
            fieldCentric = false;
            //slowmode
            if (myOpMode.gamepad1.dpad_down && !dDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            }else if (myOpMode.gamepad1.dpad_down && !dDownHigh && slowModeIsOn){
                slowModeIsOn = false;
            }
            dDownHigh = myOpMode.gamepad1.dpad_down;
        }
        if (currDriver == driverControls[1]) {//Camden
            fieldCentric = false;
            //no slow mode
        }
        if (currDriver == driverControls[2]) {//Graden
            fieldCentric = true;
            //slowmode
            if (myOpMode.gamepad1.dpad_down && !dDownHigh && !slowModeIsOn) {
                slowModeIsOn = true;
            }else if (myOpMode.gamepad1.dpad_down && !dDownHigh && slowModeIsOn){
                slowModeIsOn = false;
            }
            dDownHigh = myOpMode.gamepad1.dpad_down;

        }
        if (currDriver == driverControls[3]) {//Kian
            fieldCentric = true;
        }
        if (currDriver == driverControls[4]) {//Child
            fieldCentric = false;
        }
    }

    public void bindOtherButtons() {
        //"Chase", "Camden","Graden","Kian","Child"
        if (currOther == otherControls[0]) {//Chase
        }
        if (currOther == otherControls[1]) {//Camden
        }
        if (currOther == otherControls[2]) {//Graden
        }
        if (currOther == otherControls[3]) {//Kian
        }
        if (currOther == otherControls[4]) {//Child
        }
    }

    public void power() {// put all set power here
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void buildTelemetry() {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        //Telemetry telemetry = myOpMode.telemetry;
        telemetry.addData("potentiometer", "%.1f", getPotentVal());
        telemetry.addData("limitSwitch", getLimitSwitch());

        telemetry.addData("Drivers", currDriver + " " + currOther);
        getBatteryVoltage();
        telemetry.addData("Voltage", "%.1f", currentVoltage);//shows current battery voltage
        telemetry.addData("lowBattery", lowVoltage);
        telemetry.addData("Color", LEDcolor);
        telemetry.addData("reversed", reversed);
        telemetry.addData("slowMode", slowModeIsOn);
        //getOrientation();
        //telemetry.addData("heading", "%.1f", heading);
        //telemetry.addData("roll", "%.1f", roll);
        //telemetry.addData("pitch", "%.1f", pitch);
        //end testing
        teleSpace();
        telemetry.addData("x", "%.2f", drive.getPoseEstimate().getX());
        telemetry.addData("y", "%.2f", drive.getPoseEstimate().getY());
        telemetry.addData("heading", "%.2f", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        teleSpace();
        telemetry.addData("thisDistance (in)", "%.1f", thisDist);
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist);
        teleSpace();
        telemetry.addData("Timer", "%.1f", timer.seconds());//shows current time
        teleSpace();
        //webcam telemetry, next three lines
        //telemetry.addData("Frame Count", webcam.getFrameCount());
        //telemetry.addData("FPS", "%.2f", webcam.getFps());
        //telemetry.addData("Pipeline",pipelineName);
        //teleSpace();
        updateStatus("Running");
        telemetry.addData("Status", statusVal);//shows current status
        teleSpace();
        telemetry.addData("Version", currentVersion);
        telemetry.update();
    }

    //sensors
    public double getPotentVal() {
        return Range.clip(POTENTIOMETER_MAX / 3.3 * potentiometer.getVoltage(), POTENTIOMETER_MIN, POTENTIOMETER_MAX);
    }

    public boolean getLimitSwitch() {
        limitSwitchState = !limitSwitch.getState();
        return limitSwitchState;
    }

    void getBatteryVoltage() { //VoltageSensor sensor
        double result = Double.POSITIVE_INFINITY;
        double voltage = vSensor.getVoltage();
        if (voltage > 0) {
            result = Math.min(result, voltage);
        }
        lowVoltage = result <= minimumVoltage;
        currentVoltage = result;
    }

    //IMU
    //public void getOrientation() {
    //    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //    heading = angles.firstAngle - primaryHeading;
    //    roll = angles.secondAngle - primaryRoll;
    //    pitch = angles.thirdAngle - primaryPitch;
    //}

    //meant to fix orientation problems
    //public void findOrientationOffset() {
    //    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //    primaryHeading = angles.firstAngle;
    //    primaryRoll = angles.secondAngle;
    //    primaryPitch = angles.thirdAngle;
    //}

    //claw
    public double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    //random
    public void teleSpace() {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        //Telemetry telemetry = myOpMode.telemetry;
        telemetry.addLine(" ");
    }

    public void updateStatus(String status) {
        statusVal = status;
    }

    public void rumble() {//soccer whistle
        if ((timer.seconds() > endgame) && !isEndgame) {
            myOpMode.gamepad1.runRumbleEffect(customRumbleEffect);
            myOpMode.gamepad2.runRumbleEffect(customRumbleEffect);
            isEndgame = true;
        }
        if ((timer.seconds() > end) && !isEnd) {
            myOpMode.gamepad1.runRumbleEffect(customRumbleEffect1);
            myOpMode.gamepad2.runRumbleEffect(customRumbleEffect1);
            isEnd = true;
        }
    }

}
