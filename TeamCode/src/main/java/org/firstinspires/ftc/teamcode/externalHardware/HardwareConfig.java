//import
package org.firstinspires.ftc.teamcode.externalHardware;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotLog;
import org.firstinspires.ftc.teamcode.ggsamples.testOpModes.distanceSensorCalibrator;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class HardwareConfig {


    public static final String TFOD_MODEL_ASSET = "custom.tflite";
    public static final String[] LABELS = {
            "capacitor",//3
            "led",//1
            "resistor"//2
    };
    public static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //arm
    //other variables
    public boolean slowModeIsOn = false;//declaring the slowModeIsOn variable
    public boolean slowModeIsOn2 = false;//declaring the slowModeIsOn variable
    public boolean reversed = false;//declaring the reversed variable
    public int pitchMult = 2;
    //servo variables
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180
    public final int baseClawVal = 30;//declaring the baseClawVal variable
    public final int magicNumOpen = 60;//declaring the magicNumOpen variable
    public boolean clawOpen = false;//declaring the clawOpen variable

    //arm labels

    //public boolean limiter = false;//declaring the limiter variable, is on or off
    //public boolean limiting = false;//declaring the limiting variable
    public static final ElapsedTime runtime = new ElapsedTime();

    //rumble
    public Gamepad.RumbleEffect customRumbleEffect;//declaring the customRumbleEffect variable
    public final double endgame = 120;//declaring the endgame variable
    public boolean isEndgame = false;//declaring the isEndgame variable
    public Gamepad.RumbleEffect customRumbleEffect1;// declaring the customRumbleEffect1 variable
    public boolean rumble = false;//declaring the rumble variable
    public final double end = 150;//declaring the end variable
    public boolean isEnd = false;//declaring the isEnd variable

    //rake

    double yAxisPower;
    double zAxisPower;
    //motors
    public static DcMotor motorFrontLeft = null;
    public static DcMotor motorBackLeft = null;
    public static DcMotor motorFrontRight = null;
    public static DcMotor motorBackRight = null;
    //public DcMotor deadWheel = null;//declaring the deadWheel motor
    public DcMotor tapeMeasure = null;
    public static DcMotor yArmMotor = null;
    public static DcMotor zArmMotor = null;
    public TouchSensor touchSensor;
    public NormalizedColorSensor colorSensorR;//declaring the colorSensor variable
    public NormalizedColorSensor colorSensorL;//declaring the colorSensor variable
    public DigitalChannel red1;
    public DigitalChannel green1;
    public DigitalChannel red2;
    public DigitalChannel green2;
    public DigitalChannel red3;
    public DigitalChannel green3;
    public DigitalChannel red4;
    public DigitalChannel green4;
    public DistanceSensor rDistance;//declaring the rDistance sensor
    public DistanceSensor lDistance;//declaring the lDistance sensor
    public DistanceSensor fDistance;//declaring the fDistance sensor
    public RevBlinkinLedDriver lights;
    public Servo clawServo = null;
    public static DcMotor pitchMotor = null;
    public Servo tmServo = null;

    public int spot = 0;
    //color
    final float[] hsvValues = new float[3];//gets values for color sensor
    private final float redVal = 0;//the red value in rgb
    private final float greenVal = 0;//the green value in rgb
    private final float blueVal = 0;//the blue value in rgb
    private final String colorName = "N/A";//gets color name
    //
    public String statusVal = "OFFLINE";
    public double fDistanceVal = 0;
    public double lDistanceVal = 0;
    public double rDistanceVal = 0;
    //isRight side
    public boolean right = true;//declaring the right variable
    private final float redValR = 0;//the red value in rgb
    private final float greenValR = 0;//the green value in rgb
    private final float blueValR = 0;//the blue value in rgb
    private final float redValL = 0;//the red value in rgb
    private final float greenValL = 0;//the green value in rgb
    private final float blueValL = 0;//the blue value in rgb
    //assistance to driver
    public boolean assisting = false;
    //
    //slow mode
    public double slowMult = 3;
    public double slowPower;
    public double slowMult2 = 4;
    public double slowPower2;
    //driving
    public double xControl;
    public double yControl;
    public double frontRightPower;
    public double frontLeftPower;
    public double backRightPower;
    public double backLeftPower;
    double pitchPower;
    //
    //field centric
    double gamepadX;
    double gamepadY;
    double gamepadHypot;
    double controllerAngle;
    double robotDegree;
    double movementDegree;
    double offSet = 0;
    //
    //
    //imu
    public static BNO055IMU imu;
    public static Orientation angles;     //imu uses these to find angles and classify them
    public Acceleration gravity;    //Imu uses to get acceleration
    //
    //maintenance mode
    public TouchSensor touchSensorL;
    public TouchSensor touchSensorClaw;
    public TouchSensor touchSensorEject;
    public final int delay = 1;
    public boolean isSolid = false;
    public String color = "none";
    public boolean armUp = false;
    public boolean tapeOut = false;
    public final int timeout = 1;

    //
    //pitch servo
    public int pitchBase = 0;
    public int pitchMagic = pitchBase + 90;
    //

    public static int turn = 77;
    public static double yMult = 24;
    public static double xMult = 10;
    public static double ovrCurrX = 0;
    public static double ovrCurrY = 0;
    public static double ovrTurn = 0;
    public static final double COUNTS_PER_INCH_Side_dead = -665.08;
    public static final double COUNTS_PER_INCH_Side = -100;

    public static final double COUNTS_PER_MOTOR_REV_arm = 28;
    public static final double DRIVE_GEAR_REDUCTION_arm = 40;
    public static final double WHEEL_DIAMETER_INCHES_arm = 1.102;     // For figuring circumference
    public static final double COUNTS_PER_INCH_arm = (COUNTS_PER_MOTOR_REV_arm * DRIVE_GEAR_REDUCTION_arm) /
            (WHEEL_DIAMETER_INCHES_arm * 3.1415);
    //wheels
    public static final double COUNTS_PER_MOTOR_REV = 28;
    public static final double WHEEL_DIAMETER_MM = 96;
    public static final double DRIVE_GEAR_REDUCTION = 15;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double COUNTS_PER_MOTOR_REV_dead = 8192;
    public static final double WHEEL_DIAMETER_MM_dead = 96;
    public static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);
    public final int baseArmPosition = 0;
    public static final int armLimit = 4250;//declaring the armLimit variable
    public final int baseArm = 0;//declaring the baseArm variable
    public static final int lowPoleVal = 1570;//should be about 1/3 of arm limit
    public static final int midPoleVal = 290;//should be about 2/3 of arm limit
    public static final int fiveTallConeVal = 300;
    public static final int topPoleVal = armLimit;//should be close to armLimit


    //tape measure
    public static double tapePower;
    public static double tapeMeasureDiameter = 7.5;
    public static int tapeMeasureLength = 15 * 12;
    public static double countsPerInchTape = 10;
    public static double tickPerTapeMeasure = countsPerInchTape * tapeMeasureLength;
    public static int tmPose = 68;
    //
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // Class Members
    private final OpenGLMatrix lastLocation = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    private final boolean targetVisible = false;
    public List<VuforiaTrackable> allTrackables;

    //external
    HardwareMap hardwareMap = null;

    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public HardwareConfig(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void init(HardwareMap ahwMap) {
        updateStatus("Initializing");
        ElapsedTime runtime = new ElapsedTime();//declaring the runtime variable
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        lights = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        rDistance = ahwMap.get(DistanceSensor.class, "rDistance");//getting the rDistance sensor
        lDistance = ahwMap.get(DistanceSensor.class, "lDistance");//getting the lDistance sensor
        fDistance = ahwMap.get(DistanceSensor.class, "fDistance");//getting the fDistance sensor
        red1 = ahwMap.get(DigitalChannel.class, "red1");//getting the red1 light
        green1 = ahwMap.get(DigitalChannel.class, "green1");//getting the green1 light
        red2 = ahwMap.get(DigitalChannel.class, "red2");//getting the red2 light
        green2 = ahwMap.get(DigitalChannel.class, "green2");//getting the green2 light
        red3 = ahwMap.get(DigitalChannel.class, "red3");//getting the red3 light
        green3 = ahwMap.get(DigitalChannel.class, "green3");//getting the green3 light
        red4 = ahwMap.get(DigitalChannel.class, "red4");//getting the red4 light
        green4 = ahwMap.get(DigitalChannel.class, "green4");//getting the green4 light
        colorSensorR = ahwMap.get(NormalizedColorSensor.class, "colorSensorR");
        colorSensorL = ahwMap.get(NormalizedColorSensor.class, "colorSensorL");
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = ahwMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = ahwMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = ahwMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = ahwMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        //deadWheel = ahwMap.get(DcMotor.class, "deadWheel");//getting the deadWheel motor
        clawServo = ahwMap.get(Servo.class, "clawServo");//getting the clawServo servo
        pitchMotor = ahwMap.get(DcMotor.class, "pitchMotor");
        tmServo = ahwMap.get(Servo.class, "tmServo");//getting the tmServo servo
        yArmMotor = ahwMap.get(DcMotor.class, "yArm");
        zArmMotor = ahwMap.get(DcMotor.class, "zArm");//getting the zArm motor
        touchSensor = ahwMap.get(TouchSensor.class, ("touchSensor"));
        touchSensorL = ahwMap.get(TouchSensor.class, ("touchSensorL"));
        touchSensorClaw = ahwMap.get(TouchSensor.class, ("touchSensorClaw"));
        touchSensorEject = ahwMap.get(TouchSensor.class, ("touchSensorEject"));
        tapeMeasure = ahwMap.get(DcMotor.class, "tapeMeasure");

        yArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the sparkLong encoder
        zArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the sparkShort encoder
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontLeft encoder
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackRight encoder
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackLeft encoder
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontRight encoder
        //deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheel encoder
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheel encoder

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        tapeMeasure.setDirection(DcMotor.Direction.REVERSE);
        yArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the sparkLong encoder to run using encoder
        zArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the sparkShort encoder to run using encoder
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontLeft encoder to run using encoder
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackLeft encoder to run using encoder
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackRight encoder to run using encoder
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontRight encoder to run using encoder
        //deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheel encoder to run using encoder
        pitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tapeMeasure.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheel encoder to run using encoder

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        yArmMotor.setZeroPowerBehavior(BRAKE);
        zArmMotor.setZeroPowerBehavior(BRAKE);
        tapeMeasure.setZeroPowerBehavior(BRAKE);
        pitchMotor.setZeroPowerBehavior(BRAKE);
        red1.setMode(DigitalChannel.Mode.OUTPUT);//setting the red1 light to output
        green1.setMode(DigitalChannel.Mode.OUTPUT);//setting the green1 light to output
        red2.setMode(DigitalChannel.Mode.OUTPUT);//setting the red2 light to output
        green2.setMode(DigitalChannel.Mode.OUTPUT);//setting the green2 light to output
        red3.setMode(DigitalChannel.Mode.OUTPUT);//setting the red3 light to output
        green3.setMode(DigitalChannel.Mode.OUTPUT);//setting the green3 light to output
        red4.setMode(DigitalChannel.Mode.OUTPUT);//setting the red4 light to output
        green4.setMode(DigitalChannel.Mode.OUTPUT);//setting the green4 light to output

        //flipper.setPosition(setServo(magicFlip));//setting the flipper servo to the magicFlip position
        runtime.reset();//resetting the runtime variable
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
        if (myOpMode.isStopRequested()) {
            log((myOpMode.getClass()) + "stopped");
            return;
        }


        log("Init Done");
        if (myOpMode.isStarted()) {
            log((myOpMode.getClass()) + "started");
        }

    }

    void initTrackables(HardwareMap ahwMap) {
        webcamName = ahwMap.get(WebcamName.class, "Webcam");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters camParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        camParameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        camParameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        camParameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(camParameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(camParameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();
    }

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void doBulk(boolean fieldCentric) {
        switches();
        doClaw(false, 0);
        drive(fieldCentric, slowPower);
        runArm();
        assistArm();
        tapeMeasure();
        //
        // check all the trackable targets to see which one (if any) is visible.
        //targetVisible = false;
        //for (VuforiaTrackable trackable : allTrackables) {
        //    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
        //        myOpMode.telemetry.addData("Visible Target", trackable.getName());
        //        targetVisible = true;
        //
        //        // getUpdatedRobotLocation() will return null if no new information is available since
        //        // the last time that call was made, or if the trackable is not currently visible.
        //        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
        //        if (robotLocationTransform != null) {
        //            lastLocation = robotLocationTransform;
        //        }
        //        break;
        //    }
        //}
        //
        //// Provide feedback as to where the robot is located (if we know).
        //if (targetVisible) {
        //    // express position (translation) of robot in inches.
        //    VectorF translation = lastLocation.getTranslation();
        //    myOpMode.telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
        //            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
        //
        //    log("x" + (translation.get(0) / mmPerInch));
        //    log("y" + (translation.get(1) / mmPerInch));
        //
        //    // express the rotation of the robot in degrees.
        //    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        //    myOpMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        //} else {
        //    myOpMode.telemetry.addData("Visible Target", "none");
        //}
        power();
        buildTelemetry();
    }

    public void power() {
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        tapeMeasure.setPower(tapePower);
        //yArmMotor.setPower(yAxisPower);
        zArmMotor.setPower(zAxisPower / 2);
        pitchMotor.setPower(pitchPower / 2);
        tmServo.setPosition(setServo(tmPose));
    }

    public void log(String message) {
        RobotLog.d("logging", message);
    }

    public void runArm() {
        //yAxisPower = myOpMode.gamepad2.left_stick_y;
        zAxisPower = myOpMode.gamepad2.right_stick_y;
        pitchPower = -myOpMode.gamepad2.left_stick_y;
    }

    public void tapeMeasure() {
        if (myOpMode.gamepad2.dpad_up && (tmPose > 1)) {
            tmPose--;
        } else if (myOpMode.gamepad2.dpad_down) {
            tmPose++;
        }
        tapePower = 0;
        if ((myOpMode.gamepad1.dpad_up || myOpMode.gamepad2.dpad_right)) {// && (tapeMeasure.getCurrentPosition() < tapeLimit - tapeLimit / 5)) {
            //extend
            tapePower = 1;
        }
        if ((myOpMode.gamepad1.dpad_down || myOpMode.gamepad2.dpad_left)) {// && (tapeMeasure.getCurrentPosition() > 0 + tapeLimit / 5))) {
            //retract
            tapePower = -1;
        }
        if (tmPose > 68 + 10 || tmPose < 68 - 10) {
            green4.setState(false);
            red4.setState(true);
        } else {
            green4.setState(true);
            red4.setState(false);
        }
    }

    public void greenRed() {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        color = "RED";
        sleep(delay * 1000);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        color = "GREEN";
        isSolid = true;
    }

    //public void runOldArm() {
    //    armPower = -myOpMode.gamepad2.left_stick_y;
    //    oldPresets();
    //}
    //
    //public void oldPresets() {
    //    if (myOpMode.gamepad2.left_stick_y != 0) {
    //        armPower = -myOpMode.gamepad2.left_stick_y;
    //    } else if (myOpMode.gamepad2.y) {//top level
    //        if (sparkLong.getCurrentPosition() < topPoleVal) {//go up
    //            armPower = 1;
    //        }
    //    } else if (myOpMode.gamepad2.a) {//base
    //        if (sparkLong.getCurrentPosition() > baseArm) {//go down
    //            armPower = -1;
    //        }
    //    } else if (myOpMode.gamepad2.b) {//middle
    //        if (sparkLong.getCurrentPosition() > midPoleVal + 50) {//go down
    //            armPower = -1;
    //        }
    //        if (sparkLong.getCurrentPosition() < midPoleVal - 50) {//go up
    //            armPower = 1;
    //        }
    //    } else if (myOpMode.gamepad2.x) {//low
    //        if (sparkLong.getCurrentPosition() > lowPoleVal + 50) {//go down
    //            armPower = -1;
    //        }
    //        if (sparkLong.getCurrentPosition() < lowPoleVal - 50) {//go up
    //            armPower = 1;
    //        }
    //    }
    //}

    public void doClaw(boolean expandIfOver, int armOver) {
        //claw code
        //if (myOpMode.gamepad2.left_bumper && expandIfOver) {
        //    if (sparkLong.getCurrentPosition() > armOver) {
        //        clawServo.setPosition(setServo(magicNumOpen + 30));
        //    } else {
        //        clawServo.setPosition(setServo(magicNumOpen));
        //    }
        //    clawOpen = true;
        //    //open claw
        //} else if (myOpMode.gamepad2.right_bumper) {
        //    clawServo.setPosition(setServo(baseClawVal));
        //    //close claw
        //    clawOpen = false;
        //}
        if (myOpMode.gamepad2.left_bumper) {
            clawServo.setPosition(setServo(magicNumOpen));
            clawOpen = true;
            //open claw
        } else if (myOpMode.gamepad2.right_bumper) {
            clawServo.setPosition(setServo(baseClawVal));
            //close claw
            clawOpen = false;
        }
        if (clawOpen) {
            green1.setState(false);
            red1.setState(true);
        } else {
            green1.setState(true);
            red1.setState(false);
        }
    }

    public void switches() {
        //switches
        if (myOpMode.gamepad1.left_trigger > 0) {
            slowModeIsOn = false;
        }
        if (myOpMode.gamepad1.right_trigger > 0) {
            slowModeIsOn = true;
        }
        if (slowModeIsOn) {
            slowPower = slowMult;
            green2.setState(true);
            red2.setState(false);
        } else {
            slowPower = 1;
            green2.setState(false);
            red2.setState(true);
        }

        if (myOpMode.gamepad2.left_trigger > 0) {
            slowModeIsOn2 = false;
        }
        if (myOpMode.gamepad1.right_trigger > 0) {
            slowModeIsOn2 = true;
        }
        if (slowModeIsOn2) {
            slowPower2 = slowMult2;
            //green4.setState(true);
            //red4.setState(false);
        } else {
            slowPower2 = 1;
            green4.setState(false);
            red4.setState(true);
        }
        //
    }

    public void assistArm() {
        while (myOpMode.gamepad2.b) {

            green3.setState(false);
            red3.setState(true);
            ////put z down
            zArmEncoder(0, 0.6, 3, true);
            if (!myOpMode.gamepad2.b) {
                break;
            }
            //put pitch back
            pitchEncoder(0, 0.5, 2, true);
            if (!myOpMode.gamepad2.b) {
                break;
            }
            //let go
            openClaw();
            if (!myOpMode.gamepad2.b) {
                break;
            }
            sleep(500);
            closeClaw();
            //put pitch out
            pitchEncoder(-40, 0.5, 2, false);
            if (!myOpMode.gamepad2.b) {
                break;
            }
            //done
        }
    }

    public void drive(boolean fieldCentric, double slow) {
        if (fieldCentric) {
            gamepadX = myOpMode.gamepad1.left_stick_x;//get the x val of left stick and store
            myOpMode.telemetry.addData("gamepadX", gamepadX);//tell us what gamepadX is
            gamepadY = myOpMode.gamepad1.left_stick_y;//get the y val of left stick and store
            myOpMode.telemetry.addData("gamepadY", gamepadY);//tell us what gamepadY is
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);//get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            myOpMode.telemetry.addData("gamepadHypot", gamepadHypot);//tell us what gamepadHypot is
            controllerAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX));//Get the angle of the controller stick using arc tangent
            myOpMode.telemetry.addData("controllerAngle", controllerAngle);//tell us what controllerAngle is
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//get and initialize the IMU
            robotDegree = angles.firstAngle;//store robot angle in robotDegree
            myOpMode.telemetry.addData("robotDegree", robotDegree);//tell us what robotDegree is
            movementDegree = (controllerAngle - robotDegree);//get the movement degree based on the controller vs robot angle
            myOpMode.telemetry.addData("movementDegree", movementDegree);//tell us what movementDegree is
            xControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;//get the x value of the movement
            myOpMode.telemetry.addData("xControl", xControl);//tell us what xControl is
            yControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;//get the y value of the movement
            myOpMode.telemetry.addData("yControl", yControl);//tell us what yControl is
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower;
            backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower;
            frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower;
            backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower;
        } else {
            boolean reverse;
            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;
            yControl = -myOpMode.gamepad1.left_stick_y;
            xControl = myOpMode.gamepad1.left_stick_x;
            if (reverse) {
                yControl = -yControl;
                xControl = -xControl;
            }
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl - xControl + turn) / slow;
            backRightPower = (yControl + xControl + turn) / slow;
            frontLeftPower = (yControl + xControl - turn) / slow;
            backLeftPower = (yControl - xControl - turn) / slow;

        }
    }

    public void rumble() {
        if ((runtime.seconds() > endgame) && !isEndgame) {
            myOpMode.gamepad1.runRumbleEffect(customRumbleEffect);
            myOpMode.gamepad2.runRumbleEffect(customRumbleEffect);
            isEndgame = true;
        }
        if ((runtime.seconds() > end) && !isEnd) {
            myOpMode.gamepad1.runRumbleEffect(customRumbleEffect1);
            myOpMode.gamepad2.runRumbleEffect(customRumbleEffect1);
            isEnd = true;
        }
    }

    public void buildTelemetry() {
        myOpMode.telemetry.addData("Status", statusVal);//shows current status
        myOpMode.telemetry.addLine("Arm: ")
                .addData("y", String.valueOf(yArmMotor.getCurrentPosition()))
                .addData("z", String.valueOf(zArmMotor.getCurrentPosition()));
        myOpMode.telemetry.addData("reversed", reversed);
        myOpMode.telemetry.addData("slowMode", slowModeIsOn);
        myOpMode.telemetry.addData("arm slowMode", slowModeIsOn2);
        //myOpMode.telemetry.addData("dead", deadWheel.getCurrentPosition());
        myOpMode.telemetry.addData("tmPose", tmPose);
        myOpMode.telemetry.addData("tm", tapeMeasure.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        myOpMode.telemetry.addData("heading", angles.firstAngle);
        myOpMode.telemetry.addData("pitchMotor", pitchMotor.getCurrentPosition());
        myOpMode.telemetry.addData("tape power", tapePower);
        myOpMode.telemetry.addLine("motors: ")
                .addData("front left", motorFrontLeft.getCurrentPosition())
                .addData("front right", motorFrontRight.getCurrentPosition())
                .addData("back left", motorBackLeft.getCurrentPosition())
                .addData("back right", motorBackRight.getCurrentPosition());
        myOpMode.telemetry.addLine("power: ")
                .addData("front left", frontLeftPower)
                .addData("front right", frontRightPower)
                .addData("back left", backLeftPower)
                .addData("back right", backRightPower);
        teleSpace();
        updateStatus("Running");
        myOpMode.telemetry.update();
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

    public String getColorFrequencyTest() {
        double commonP = 0.2;
        double uncommonP = 0.1;
        double rareP = 0.057;
        double pOne;
        double pTwo;
        double pThree;
        double pFour;
        double pFive;
        double pSix;
        double pSeven;
        double pEight;
        double pNine;
        double pTen;
        double pEleven;
        pOne = pTwo = pThree = pFour = pFive = pNine = pEleven = rareP;//7 * rareP  = 0.399
        pSix = pTen = uncommonP;//2 * uncommonP = 0.2
        pSeven = pEight = commonP;//2 * commonP = 0.4

        double[] p = {pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine, pTen, pEleven};
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",//1
                "RAINBOW_PARTY_PALETTE",//2
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",//3
                "BEATS_PER_MINUTE_PARTY_PALETTE",//4
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
        //final int min = 0;
        //final int max = favColors.length - 1;
        //double ran = (int) Math.floor(Math.random() * (max - min + 1) + min);
        int min = 0;
        int max = 100;
        double ran = (int) Math.floor(Math.random() * (max - min + 1) + min);
        if (ran < p[1] && ran > 0) {
            return favColors[0];
        } else if (ran < p[2] && ran > p[1]) {
            return favColors[1];
        } else if (ran < p[3] && ran > p[2]) {
            return favColors[2];
        } else if (ran < p[4] && ran > p[3]) {
            return favColors[3];
        } else if (ran < p[5] && ran > p[4]) {
            return favColors[4];
        } else if (ran < p[6] && ran > p[5]) {
            return favColors[5];
        } else if (ran < p[7] && ran > p[6]) {
            return favColors[6];
        } else if (ran < p[8] && ran > p[7]) {
            return favColors[7];
        } else if (ran < p[9] && ran > p[8]) {
            return favColors[8];
        } else if (ran < p[10] && ran > p[9]) {
            return favColors[9];
        } else {
            return favColors[10];
        }
    }

    //random
    public void teleSpace() {
        myOpMode.telemetry.addLine();
    }

    public void updateStatus(String status) {
        statusVal = status;
    }//set a new controller/game status




    //
//claw
    public double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public void openClaw() {
        clawServo.setPosition(setServo(magicNumOpen));
    }

    public void closeClaw() {
        clawServo.setPosition(setServo(baseClawVal));
    }

    //
//encoder
    public void tapeEncoder(int pose, double speed, double timeOut, boolean isOut) {
        tapeMeasure.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tapeMeasure.setTargetPosition(pose);
        tapeMeasure.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (isOut) {
            tapeMeasure.setPower(-speed);//go in
        }
        if (!isOut) {
            tapeMeasure.setPower(speed);//go out
        }
        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeOut) && tapeMeasure.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", pose);
            myOpMode.telemetry.addData("Currently at",
                    tapeMeasure.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        tapeMeasure.setPower(0);
        tapeMeasure.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.update();
    }


    //public void encoderComboFwd(double speed, double lInches, double rInches,
    //                            double pose, double timeoutS, boolean isUp) {
    //    int newLeftTarget;
    //    int newRightTarget;
    //    int newDLeftTarget;
    //    int newDRightTarget;
    //    int target;
    //    target = (int) pose;
    //    zArmMotor.setTargetPosition(target);
    //    zArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //
    //    newLeftTarget = motorBackLeft.getCurrentPosition() + (int) (lInches * COUNTS_PER_INCH);
    //    newRightTarget = motorBackRight.getCurrentPosition() + (int) (rInches * COUNTS_PER_INCH);
    //    //newDLeftTarget = deadWheelL.getCurrentPosition() + (int) (lInches * COUNTS_PER_INCH_dead);
    //    //newDRightTarget = deadWheelR.getCurrentPosition() + (int) (rInches * COUNTS_PER_INCH_dead);
    //
    //    //deadWheelL.setTargetPosition(-newDLeftTarget);
    //    //deadWheelR.setTargetPosition(-newDRightTarget);
    //    motorFrontRight.setTargetPosition(-newRightTarget);
    //    motorBackRight.setTargetPosition(-newRightTarget);
    //    motorFrontLeft.setTargetPosition(-newLeftTarget);
    //    motorBackLeft.setTargetPosition(-newLeftTarget);
    //
    //    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //    //deadWheelL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //    //deadWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //
    //    runtime.reset();
    //    if (isUp) {
    //        sparkLong.setPower(speed);//go down
    //    }
    //    if (!isUp) {
    //        sparkLong.setPower(-speed);
    //    }
    //    motorBackLeft.setPower((speed));
    //    motorBackRight.setPower((speed));
    //    motorFrontRight.setPower((speed));
    //    motorFrontLeft.setPower((speed));
    //    while (myOpMode.opModeIsActive() &&
    //            (runtime.seconds() < timeoutS) && sparkLong.isBusy()) {
    //
    //        // Display it for the driver.
    //        myOpMode.telemetry.addData(" arm Running to", sparkLong.getCurrentPosition());
    //        myOpMode.telemetry.addData("arm Currently at",
    //                sparkLong.getCurrentPosition());
    //        // Display it for the driver.
    //        myOpMode.telemetry.update();
    //    }
    //
    //    // Stop all motion;
    //    motorBackLeft.setPower(0);
    //    motorFrontRight.setPower(0);
    //    motorBackRight.setPower(0);
    //    motorFrontLeft.setPower(0);
    //
    //    sparkLong.setPower(0);
    //    sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    // Turn off RUN_TO_POSITION
    //    motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    //deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    //deadWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    resetEncoders();
    //    myOpMode.telemetry.update();
    //}

    public static void encoderDrive(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newDLeftTarget;
        int newDRightTarget;
        if (myOpMode.opModeIsActive()) {

            newLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            //newDLeftTarget = deadWheelL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_dead);
            //newDRightTarget = deadWheelR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_dead);

            //deadWheelL.setTargetPosition(-newDLeftTarget);
            //deadWheelR.setTargetPosition(-newDRightTarget);
            motorFrontRight.setTargetPosition(-newRightTarget);
            motorBackRight.setTargetPosition(-newRightTarget);
            motorFrontLeft.setTargetPosition(-newLeftTarget);
            motorBackLeft.setTargetPosition(-newLeftTarget);

            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //deadWheelL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //deadWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorBackLeft.setPower((speed));
            motorFrontRight.setPower((speed));
            motorFrontLeft.setPower((speed));
            motorBackRight.setPower((speed));
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorBackLeft.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Running to", "%7d :%7d", -newDLeftTarget, -newDRightTarget);//"%7d :%7d"
                //telemetry.addData("Currently at", "%7d :%7d",
                //        deadWheelL.getCurrentPosition(), deadWheelR.getCurrentPosition());
                myOpMode.telemetry.addData("fr", motorFrontRight.getCurrentPosition());
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //deadWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoders();
        }
    }

    public static void sideWaysEncoderDrive(double speed,
                                            double inches,
                                            double timeoutS) {//+=right //-=left
        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;
        int newDeadTarget;
        inches *= -1;
        if (myOpMode.opModeIsActive()) {
            if (inches < 0) {
                newFLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = motorBackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                //newDeadTarget = deadWheel.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side_dead);
                motorFrontLeft.setTargetPosition(-newFLTarget);
                motorBackLeft.setTargetPosition(newBLTarget);
                motorBackRight.setTargetPosition(-newBRTarget);
                motorFrontRight.setTargetPosition(newFRTarget);
                //deadWheel.setTargetPosition(-newDeadTarget);
            } else if (inches > 0) {
                newFLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = motorBackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                //newDeadTarget = deadWheel.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side_dead);
                motorFrontLeft.setTargetPosition(-newFLTarget);
                motorBackLeft.setTargetPosition(newBLTarget);
                motorBackRight.setTargetPosition(-newBRTarget);
                motorFrontRight.setTargetPosition(newFRTarget);
                //deadWheel.setTargetPosition(newDeadTarget);
            } else {
                motorFrontLeft.setTargetPosition(0);
                motorBackLeft.setTargetPosition(0);
                motorBackRight.setTargetPosition(0);
                motorFrontRight.setTargetPosition(0);
                //deadWheel.setTargetPosition(0);
            }

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //deadWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorBackLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && motorFrontLeft.isBusy()) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", "%7d:%7d", motorFrontLeft.getCurrentPosition()
                        , motorBackRight.getCurrentPosition());
                myOpMode.telemetry.addData("Running to", "%7d:%7d", motorBackLeft.getCurrentPosition()
                        , motorFrontRight.getCurrentPosition());
                myOpMode.telemetry.addData("Currently at", "%7d:%7d",
                        motorFrontLeft.getCurrentPosition()
                        , motorBackRight.getCurrentPosition());
                myOpMode.telemetry.addData("Currently at", "%7d:%7d",
                        motorFrontRight.getCurrentPosition()
                        , motorBackLeft.getCurrentPosition());
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoders();
        }
    }

    public void pitchEncoder(double pose, double speed, double timeOut, boolean isDown) {
        int target;
        target = (int) pose;
        pitchMotor.setTargetPosition(target);
        pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (isDown) {
            pitchMotor.setPower(speed);//go down
        }
        if (!isDown) {
            pitchMotor.setPower(-speed);
        }
        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeOut) && pitchMotor.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", pitchMotor.getCurrentPosition());
            myOpMode.telemetry.addData("Currently at",
                    pitchMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        pitchMotor.setPower(0);
        pitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.update();
    }

    public void yArmEncoder(double pose, double speed, double timeOut, boolean isUp) {
        int target;
        target = (int) pose;
        yArmMotor.setTargetPosition(target);
        yArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (isUp) {
            yArmMotor.setPower(speed);//go down
        }
        if (!isUp) {
            yArmMotor.setPower(-speed);
        }
        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeOut) && yArmMotor.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", yArmMotor.getCurrentPosition());
            myOpMode.telemetry.addData("Currently at",
                    yArmMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        yArmMotor.setPower(0);
        yArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.update();
    }

    public void zArmEncoder(double pose, double speed, double timeOut, boolean isUp) {
        int target;
        target = (int) pose;
        zArmMotor.setTargetPosition(target);
        zArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (isUp) {
            zArmMotor.setPower(speed);//go down
        }
        if (!isUp) {
            zArmMotor.setPower(-speed);
        }
        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeOut) && zArmMotor.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", zArmMotor.getCurrentPosition());
            myOpMode.telemetry.addData("Currently at",
                    zArmMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        zArmMotor.setPower(0);
        zArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.update();
    }

    public static void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //
//distance

    //
//walmart odo
    public void turn(int degrees) {
        resetEncoders();
        if (degrees > 180) {
            degrees = (360 - degrees) * -1;
        }
        if (degrees <= 0) {
            degrees += 1;
        }
        int mult = 360 / (degrees + 1);
        int inches = (turn / mult);
        encoderDrive(0.65, -inches, inches, 3);
        resetEncoders();
    }

    public void advGoSpot(double currX, double currY, double targetX, double targetY, double power, boolean combo, int pose
            , boolean isUp, String orientation, double orientationVal, boolean endTurn, int turn) {
        if (ovrTurn % 180 == 0) {
            double mult = ovrTurn / 180;
            orientationVal *= (Math.pow(-1, mult));
        }
        if (Objects.equals(orientation, "|")) {
            double sidewaysInches = (targetX - currX) * xMult;
            double fwdInches = (targetY - currY) * yMult;
            fwdInches *= orientationVal;
            sidewaysInches *= orientationVal;
            myOpMode.telemetry.addData("fwdInches", fwdInches);
            myOpMode.telemetry.addData("sidewaysInches", sidewaysInches);
            if (currX < targetX) {
                sideWaysEncoderDrive(power, sidewaysInches, 1);
            } else if (currX > targetX) {
                sideWaysEncoderDrive(power, -sidewaysInches, 1);
            }
            if (currY < targetY) {
                if (!combo) {
                    encoderDrive(power, fwdInches, fwdInches, 6);
                } else {
                    encoderDrive(power, fwdInches, fwdInches, 6);
                    yArmEncoder(pose, power, 6, isUp);
                }
            } else if (currY > targetY) {
                if (!combo) {
                    encoderDrive(power, -fwdInches, -fwdInches, 6);
                } else {
                    encoderDrive(power, -fwdInches, -fwdInches, 6);
                    yArmEncoder(pose, power, 1, isUp);
                }
            }
        }
        if (endTurn) {
            turn(turn);
        }
        ovrTurn += turn;
        //telemetry.addData("orientation", orientation);
        myOpMode.telemetry.update();
        //sleep(5000);
    }

    //
//vu
    public void runVu(int timeoutS, boolean giveSpot) {
        runtime.reset();
        while (myOpMode.opModeIsActive() && (spot == 0)) {
            if (runtime.seconds() > timeoutS) {
                spot = 2;
            }
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    myOpMode.telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        myOpMode.telemetry.addData("", " ");
                        myOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        myOpMode.telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        myOpMode.telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        if (giveSpot && spot == 0) {
                            if (Objects.equals(recognition.getLabel(), "led")) {
                                spot += 1;
                                break;
                            }
                            if (Objects.equals(recognition.getLabel(), "resistor")) {
                                spot += 2;
                                break;
                            }
                            if (Objects.equals(recognition.getLabel(), "capacitor")) {
                                spot += 3;
                                break;
                            }
                        }
                    }
                    myOpMode.telemetry.update();
                }
            }
        }
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //
//color
    public boolean colorInRange(float red, double targetR, float green, double targetG, float blue, double targetB, float range) {
        boolean rCheck = false;
        boolean gCheck = false;
        boolean bCheck = false;
        if (targetR - range < red && red < targetR + range) {
            rCheck = true;
        }
        if (targetG - range < green && green < targetG + range) {
            gCheck = true;
        }
        if (targetB - range < blue && blue < targetB + range) {
            bCheck = true;
        }
        return rCheck && gCheck && bCheck;
    }

    public void getAllColorR() {
        //gives color values
        NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
        Color.colorToHSV(colorsR.toColor(), hsvValues);
        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colorsR.red)
                .addData("Green", "%.3f", colorsR.green)
                .addData("Blue", "%.3f", colorsR.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colorsR.alpha);
        myOpMode.telemetry.addLine()
                .addData("Color", colorName)
                .addData("RGB", "(" + redValR + "," + greenValR + "," + blueValR + ")");//shows rgb value
    }

    public void getAllColorL() {
        //gives color values
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colors.alpha);
        myOpMode.telemetry.addLine()
                .addData("Color", colorName)
                .addData("RGB", "(" + redValL + "," + greenValL + "," + blueValL + ")");//shows rgb value
    }
//
}
