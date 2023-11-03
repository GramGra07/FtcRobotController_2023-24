//import
package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.Drivers.bindDriverButtons;
import static org.firstinspires.ftc.teamcode.Drivers.currDriver;
import static org.firstinspires.ftc.teamcode.Drivers.currOther;
import static org.firstinspires.ftc.teamcode.Drivers.fieldCentric;
import static org.firstinspires.ftc.teamcode.Drivers.switchProfile;
import static org.firstinspires.ftc.teamcode.Operator.bindOtherButtons;
import static org.firstinspires.ftc.teamcode.Sensors.currentVoltage;
import static org.firstinspires.ftc.teamcode.Sensors.getBatteryVoltage;
import static org.firstinspires.ftc.teamcode.Sensors.lowVoltage;
import static org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.setUpFile;
import static org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.writeToFile;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.resetEncoder;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.runWithoutEncoder;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.setDirectionR;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.zeroPowerBrake;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.baseServo;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.setServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.variable;
import org.firstinspires.ftc.teamcode.opModes.configVars.varConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.DistanceStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;

import java.io.FileWriter;

public class HardwareConfig {//this is an external opMode that can have public variables used by everything
    public static boolean useFileWriter = variable.useFileWriter;
    public static boolean multipleDrivers = variable.multipleDrivers;
    public String statusVal = "OFFLINE";
    public static Servo claw1 = null, claw2 = null, flipServo = null;
    public static DcMotor motorFrontLeft = null, motorBackLeft = null, motorFrontRight = null, motorBackRight = null, motorLift = null, motorSlides = null, motorFlipper = null;
    public static DcMotor enc1 = null;
    public static RevBlinkinLedDriver lights;
    public int slowMult = varConfig.slowMult, slowPower;
    public static boolean slowModeIsOn = false, reversed;
    public double xControl, yControl, frontRightPower, frontLeftPower, backRightPower, backLeftPower;
    public static double liftPower = 0, slidePower = 0, flipperPower = 0;
    public static double flipperMin =-1, flipperMax = 1;
    public static double liftMax = 1, liftMin = -0.7;
    public static double slideMax = 1, slideMin = -1;
    public static boolean airplaneArmed = variable.airplaneArmed;
    public static Gamepad.RumbleEffect cRE;
    double gamepadX, gamepadY, gamepadHypot, controllerAngle, robotDegree, movementDegree;
    boolean reverse = false;
    public int delay = varConfig.delay;
    public boolean isSolid = false;
    public static String LEDcolor;
    public static DigitalChannel green1;
    public static DigitalChannel green2;
    public static DigitalChannel green3;
    public static DigitalChannel green4;
    public static DigitalChannel red1;
    public static DigitalChannel red2;
    public static DigitalChannel red3;
    public static DigitalChannel red4;
    public AnalogInput potentiometer;
    public DigitalChannel limitSwitch;
    public VoltageSensor vSensor;
    public SampleMecanumDrive drive = null;
    public static double thisDist = 0;
    public static final ElapsedTime timer = new ElapsedTime();
    FileWriter fileWriter;
    HardwareMap hardwareMap = null;

    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public HardwareConfig(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public boolean once = false;
    public final String currentVersion = "4.1.0";

    //init
    public void init(HardwareMap ahwMap) {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        thisDist = 0;
        setUpFile(fileWriter);
        updateStatus("Initializing");
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        ElapsedTime timer = new ElapsedTime();//declaring the runtime variable
        vSensor = ahwMap.voltageSensor.get("Expansion Hub 2");//getting the voltage sensor
        getBatteryVoltage(vSensor);
        lights = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        // rev potentiometer //analog
        potentiometer = ahwMap.get(AnalogInput.class, "potent");
        //magnetic limit switch //digital is pressed
        limitSwitch = ahwMap.get(DigitalChannel.class, "limitSwitch");
//        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        green1 = ahwMap.get(DigitalChannel.class, "green1");
        green2 = ahwMap.get(DigitalChannel.class, "green2");
        green3 = ahwMap.get(DigitalChannel.class, "green3");
        green4 = ahwMap.get(DigitalChannel.class, "green4");
        red1 = ahwMap.get(DigitalChannel.class, "red1");
        red2 = ahwMap.get(DigitalChannel.class, "red2");
        red3 = ahwMap.get(DigitalChannel.class, "red3");
        red4 = ahwMap.get(DigitalChannel.class, "red4");
        green1.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);
        green3.setMode(DigitalChannel.Mode.OUTPUT);
        green4.setMode(DigitalChannel.Mode.OUTPUT);
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        red3.setMode(DigitalChannel.Mode.OUTPUT);
        red4.setMode(DigitalChannel.Mode.OUTPUT);
        // Declare our motors
        motorFrontLeft = ahwMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = ahwMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = ahwMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = ahwMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        motorLift = ahwMap.get(DcMotor.class, "lift");
        motorSlides = ahwMap.get(DcMotor.class, "slideMotor");
        motorFlipper = ahwMap.get(DcMotor.class, "flipperMotor");
        claw1 = ahwMap.get(Servo.class, "claw1");
        claw2 = ahwMap.get(Servo.class, "claw2");
        flipServo = ahwMap.get(Servo.class, "flipServo");
        //encoders
        enc1 = ahwMap.get(DcMotor.class, "intakeMotor");
        resetEncoder(enc1);
        runWithoutEncoder(enc1);
//        runWithoutEncoder(motorPaperAirplane);
        //reversals
        setDirectionR(motorBackLeft);
        setDirectionR(motorLift);
        //set all to brake when set 0 power
        zeroPowerBrake(motorBackRight);
        zeroPowerBrake(motorBackLeft);
        zeroPowerBrake(motorFrontLeft);
        zeroPowerBrake(motorFrontRight);
        zeroPowerBrake(motorSlides);
        claw1.setPosition(setServo(baseServo));
        claw2.setPosition(setServo(baseServo));
        cRE = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .build();
        timer.reset();//resetting the runtime variable
        Sensors.ledIND(green1, red1, true);
        Sensors.ledIND(green2, red2, true);
        Sensors.ledIND(green3, red3, true);
        Sensors.ledIND(green4, red4, true);
        Blink.setLights(LEDcolor, true);
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
        once(myOpMode);//runs once
        bindDriverButtons(myOpMode);
        bindOtherButtons(myOpMode);
        if (multipleDrivers) {
            switchProfile(myOpMode);
        }
        drive(fieldCentric);
        power();//sets power to power variables
        buildTelemetry();//makes telemetry
    }

    public void once(OpMode myOpMode) {
        if (!once) {
            Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
            // Telemetry telemetry = myOpMode.telemetry;
            telemetry.clearAll();
            updateStatus("Running");
            int duration = 500;
            myOpMode.gamepad1.setLedColor(229, 74, 161,duration);
            myOpMode.gamepad2.setLedColor(54,69,79,duration);
            once = true;
        }
    }

    public void drive(boolean fieldCentric) {
        if (fieldCentric) {
            gamepadX = myOpMode.gamepad1.left_stick_x;//get the x val of left stick and store
            gamepadY = -myOpMode.gamepad1.left_stick_y;//get the y val of left stick and store
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);//get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX));//Get the angle of the controller stick using arc tangent
            robotDegree = Math.toDegrees(drive.getPoseEstimate().getHeading());
            movementDegree = (controllerAngle - robotDegree);//get the movement degree based on the controller vs robot angle
            xControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;//get the x value of the movement
            yControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;//get the y value of the movement
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
            if (slowModeIsOn) {
                slowPower = slowMult;
            } else {
                slowPower = 1;
            }
            frontRightPower = (yControl - xControl + turn) / slowPower;
            backRightPower = (yControl + xControl + turn) / slowPower;
            frontLeftPower = (yControl + xControl - turn) / slowPower;
            backLeftPower = (yControl - xControl - turn) / slowPower;
        }
        drive.update();
        updateDistTraveled(PoseStorage.currentPose, drive.getPoseEstimate());
        writeToFile(fileWriter, (int) drive.getPoseEstimate().getX(), (int) drive.getPoseEstimate().getY());
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void updateDistTraveled(Pose2d before, Pose2d after) {
        double x = after.getX() - before.getX();
        double y = after.getY() - before.getY();
        double dist = Math.sqrt((x * x) + (y * y));
        thisDist += dist;
        DistanceStorage.totalDist += dist;
    }

    public void power() {// put all set power here
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        motorLift.setPower(liftPower);
        motorSlides.setPower(slidePower);
        motorFlipper.setPower(flipperPower);
    }

    public void buildTelemetry() {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        //Telemetry telemetry = myOpMode.telemetry;

        telemetry.addData("Drivers", currDriver + " " + currOther);
        getBatteryVoltage(vSensor);
        telemetry.addData("Voltage", "%.1f", currentVoltage);//shows current battery voltage
        telemetry.addData("lowBattery", lowVoltage);
        telemetry.addData("Color", LEDcolor);
        telemetry.addData("reversed", reversed);
        telemetry.addData("slowMode", slowModeIsOn);
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
        updateStatus("Running");
        telemetry.addData("Status", statusVal);//shows current status
        teleSpace();
        telemetry.addData("Version", currentVersion);
        telemetry.update();
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

}
