package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.teleOp.robotCentric;

import java.util.Locale;

@Autonomous(name = "advAutoL", group = "Robot")
//@Disabled
public class advAutoL extends robotCentric {
    public int turn = 77;


    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double WHEEL_DIAMETER_MM = 96;
    static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * 15) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_MOTOR_REV_dead = 8192;
    static final double WHEEL_DIAMETER_MM_dead = 96;
    static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);

    static final double ROBOT_DIAMETER = 13.05;
    //arm
    final int baseArmPosition = 0;
    public final int armLimit = robotCentric.armLimit;
    public final int lowPoleVal = robotCentric.lowPoleVal;//should be about 1/3 of arm limit
    public final int midPoleVal = robotCentric.midPoleVal;//should be about 2/3 of arm limit
    public final int topPoleVal = armLimit;//should be close to armLimit
    static final double COUNTS_PER_MOTOR_REV_arm = 28;
    static final double DRIVE_GEAR_REDUCTION_arm = 40;
    static final double WHEEL_DIAMETER_INCHES_arm = 1.102;     // For figuring circumference
    static final double COUNTS_PER_INCH_arm = (COUNTS_PER_MOTOR_REV_arm * DRIVE_GEAR_REDUCTION_arm) /
            (WHEEL_DIAMETER_INCHES_arm * 3.1415);

    static final double COUNTS_PER_INCH_Side_dead = -665.08;
    static final double COUNTS_PER_INCH_Side = -100;
    public final int baseClawVal = 30;
    public final int magicNumOpen = 60;
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180

    private static final String TFOD_MODEL_ASSET = "custom.tflite";

    private static final String[] LABELS = {
            "capacitor",//3
            "led",//1
            "resistor"//2
    };

    private static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public int spot = 0;
    public double IN_distanceR = 0;//in distance for distance sensor 1
    public double IN_distanceL = 0;
    public double myMagic = 7;
    private DigitalChannel red2;
    private DigitalChannel green2;
    //color
    final float[] hsvValues = new float[3];//gets values for color sensor
    private final float redValR = 0;//the red value in rgb
    private final float greenValR = 0;//the green value in rgb
    private final float blueValR = 0;//the blue value in rgb
    private final float redValL = 0;//the red value in rgb
    private final float greenValL = 0;//the green value in rgb
    private final float blueValL = 0;//the blue value in rgb
    private final String colorName = "N/A";//gets color name
    public NormalizedColorSensor colorSensorR;//declaring the colorSensorR variable
    public NormalizedColorSensor colorSensorL;//declaring the colorSensorR variable
    public TouchSensor touchSensor;
    public TouchSensor touchSensorL;
    public boolean touchPressed = false;
    public double ovrCurrX = 2;
    public double ovrCurrY = 1;
    private final boolean bypassCam = true;
    public RevBlinkinLedDriver lights;
    public DigitalChannel red1;
    public DigitalChannel green1;
    public DigitalChannel red3;
    public DigitalChannel green3;
    public BNO055IMU imu;    //imu module inside expansion hub
    public Orientation angles;     //imu uses these to find angles and classify them
    public Acceleration gravity;    //Imu uses to get acceleration

    @Override
    public void runOpMode() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");
        lDistance = hardwareMap.get(DistanceSensor.class, "lDistance");
        fDistance = hardwareMap.get(DistanceSensor.class, "fDistance");
        red1 = hardwareMap.get(DigitalChannel.class, "red1");
        green1 = hardwareMap.get(DigitalChannel.class, "green1");
        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");
        red3 = hardwareMap.get(DigitalChannel.class, "red3");
        green3 = hardwareMap.get(DigitalChannel.class, "green3");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorR");
        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        deadWheel = hardwareMap.get(DcMotor.class, "deadWheel");
        //eadWheelL = hardwareMap.get(DcMotor.class, "deadWheelL");
        //eadWheelR = hardwareMap.get(DcMotor.class, "deadWheelR");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");
        touchSensor = hardwareMap.get(TouchSensor.class, ("touchSensor"));
        touchSensorL = hardwareMap.get(TouchSensor.class, ("touchSensorL"));

        //onInit();
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //deadWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        sparkLong.setZeroPowerBehavior(BRAKE);
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        green1.setMode(DigitalChannel.Mode.OUTPUT);
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);
        red3.setMode(DigitalChannel.Mode.OUTPUT);
        green3.setMode(DigitalChannel.Mode.OUTPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Starting at", "%7d :%7d",
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
        telemetry.update();
        closeClaw();
        // Wait for the game to start (driver presses PLAY)
        sleep(300);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        waitForStart();
        if (opModeIsActive()) {
            double ovrPower = 0.55;
            doSetup();
            encoderDrive(1, 1, 1, 0.5);
            //branch 1 get to spot
            simplerGoSpot(ovrCurrX, ovrCurrY, 3, 3, ovrPower, false, 0, false
                    , false, 0, 1, 4);
            setOvr(3, 3);
            double targetX = 2.2;
            double targetY2 = 3.5;// at pole
            simplerGoSpot(ovrCurrX, ovrCurrY, targetX, targetY2, ovrPower, false, topPoleVal,
                    false, true, 90, 2, 1);
            encoderDrive(1, 2, 2, 0.5);
            armEncoder(topPoleVal, 1, 3, false);
            setOvr(targetX, targetY2);
            openClaw();
            sleep(200);
            closeClaw();
            double targetY1 = 2.5;//lined up with cones
            simpleGoSpotRight(ovrCurrX, ovrCurrY, 0.5, targetY1, ovrPower, true, midPoleVal + 500,
                    true, false, 0, 1, 1, true);
            setOvr(3.5, targetY1);
            // Branch 2 place first cone
            correctToCones();
            double speed = 0.4;
            motorBackLeft.setPower(speed);
            motorBackRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
            armEncoder(fiveTallConeVal + 500, 1, 2, true);
            openClaw();
            armEncoder(fiveTallConeVal, 1.0, 0.5, true);
            sleep(500);
            closeClaw();
            speed = 0;
            motorBackLeft.setPower(speed);
            motorBackRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
            armEncoder(midPoleVal + 1000, 1, 2, false);//clear gap
            //vars
            //branch 3 get to stack
            simpleGoSpotRight(ovrCurrX, ovrCurrY, targetX, targetY2, ovrPower, true, topPoleVal,
                    false, false, 0, 2, 4, false);
            encoderDrive(1, 2, 2, 0.5);
            setOvr(targetX, targetY2);
            openClaw();
            simpleGoSpotRight(ovrCurrX, ovrCurrY, targetX, targetY1, ovrPower, true, midPoleVal + 500,
                    true, false, 0, 0.2, 1, true);
            setOvr(targetX, targetY1);
            closeClaw();
            //2,3
            double stackDist = 19;
            armEncoder(0, 1, 2, true);
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
                encoderDrive(1, -15, -15, 3);
                //1,3
            }
            telemetry.update();
        }
    }

    public void correctByImu(float currentAngle, int targetAngle) {
        int angle = (int) (targetAngle - currentAngle);
        turn(angle);
    }

    public void correctToCones() {
        correctByColor();
        correctByTouch();
    }

    public void doSetup() {
        runVu(6, true);
        if (spot == 0) {
            spot = (int) (Math.floor(Math.random() * (3) + 1));
        }
        if (spot == 1) {
            green1.setState(true);
            red1.setState(false);
        } else if (spot == 2) {
            green2.setState(true);
            red2.setState(false);
        } else if (spot == 3) {
            green3.setState(true);
            red3.setState(false);
        } else {
            green1.setState(false);
            red1.setState(false);
            green2.setState(false);
            red2.setState(false);
            green3.setState(false);
            red3.setState(false);
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
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


    public void getAllColorR() {
        //gives color values
        NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
        Color.colorToHSV(colorsR.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colorsR.red)
                .addData("Green", "%.3f", colorsR.green)
                .addData("Blue", "%.3f", colorsR.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colorsR.alpha);
        telemetry.addLine()
                .addData("Color", colorName)
                .addData("RGB", "(" + redValR + "," + greenValR + "," + blueValR + ")");//shows rgb value
    }

    public void getAllColorL() {
        //gives color values
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colors.alpha);
        telemetry.addLine()
                .addData("Color", colorName)
                .addData("RGB", "(" + redValL + "," + greenValL + "," + blueValL + ")");//shows rgb value
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
            encoderDrive(1, 6, 6, 1);
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public String getColor() {
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",
                "RAINBOW_PARTY_PALETTE",
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",
                "BEATS_PER_MINUTE_PARTY_PALETTE",
                //"FIRE_MEDIUM",
                "COLOR_WAVES_RAINBOW_PALETTE",
                "COLOR_WAVES_PARTY_PALETTE",
                "CP2_END_TO_END_BLEND_TO_BLACK",
                "CP2_BREATH_SLOW",
                "CP1_2_END_TO_END_BLEND_1_TO_2",
                "CP1_2_END_TO_END_BLEND",
                "HOT_PINK",
                "GOLD",
                "VIOLET"
        };
        final int min = 0;
        final int max = favColors.length - 1;
        return favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
    }

    public int refreshHeading(float usedAngle, double alterHeading) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int trueHeading = (int) ((int) usedAngle - alterHeading);
        if (trueHeading < 0) {
            trueHeading = 360 + trueHeading;
        }
        telemetry.addData("heading", trueHeading);
        telemetry.addData("usedAngle", usedAngle);
        telemetry.addData("alterHeading", alterHeading);
        telemetry.update();
        return -trueHeading;
    }

    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void simplerGoSpot(double currX, double currY, double targetX, double targetY, double power, boolean combo, int pose
            , boolean isUp, boolean endTurn, int turn, int timeOutX, int timeOutY) {
        double sidewaysInches = (targetX - currX) * xMult;
        double fwdInches = (targetY - currY) * yMult;
        telemetry.addData("fwdInches", fwdInches);
        telemetry.addData("sidewaysInches", sidewaysInches);
        sideWaysEncoderDrive(power, sidewaysInches, timeOutX);
        if (!combo) {
            encoderDrive(power, fwdInches, fwdInches, timeOutY);
        } else {
            encoderComboFwd(power, fwdInches, fwdInches, pose, timeOutY, isUp);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        telemetry.update();
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
            encoderComboFwd(power, fwdInches, fwdInches, pose, timeOutX, isUp);
        }
        if (!prioritizeY) {
            sleep(100);
            sideWaysEncoderDrive(power, sidewaysInches, timeOutY);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        telemetry.update();
    }
}
