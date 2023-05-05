//import
package org.firstinspires.ftc.teamcode.externalHardware;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class HardwareConfig {//this is an external opMode that can have public variables used by everything
    //my personal key
    public static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //other variables

    public String statusVal = "OFFLINE";
    //servo variables
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180
    public static final ElapsedTime timer = new ElapsedTime();

    //rumble
    public Gamepad.RumbleEffect customRumbleEffect, customRumbleEffect1;//declaring the customRumbleEffect variable
    public final double endgame = 120, end = 150;//declaring the endgame variable
    public boolean isEndgame = false, isEnd = false;//declaring the isEndgame variable

    //motors
    public static DcMotor motorFrontLeft = null, motorBackLeft = null, motorFrontRight = null, motorBackRight = null;

    public RevBlinkinLedDriver lights;
    //slow mode
    public double slowMult = 3, slowPower;
    public boolean slowModeIsOn = false, reversed;
    //driving
    public double xControl, yControl, frontRightPower, frontLeftPower, backRightPower, backLeftPower;
    //field centric
    double gamepadX, gamepadY, gamepadHypot, controllerAngle, robotDegree, movementDegree, offSet = 0;
    //imu
    public static BNO055IMU imu;
    public static Orientation angles;     //imu uses these to find angles and classify them
    public Acceleration gravity;    //Imu uses to get acceleration
    public double primaryHeading, primaryRoll,primaryPitch;
    public double heading,roll,pitch;
    //maintenance mode
    public final int delay = 1;
    public boolean isSolid = false;
    public String LEDcolor = "none";
    //encoder vals
    public static int turn = 77;
    public static double yMult = 24, xMult = 10;
    public static double ovrCurrX = 0, ovrCurrY = 0, ovrTurn = 0;
    public static final double COUNTS_PER_INCH_Side_dead = -665.08, COUNTS_PER_INCH_Side = -100;
    //reg mecanum
    public static final double COUNTS_PER_MOTOR_REV = 28, WHEEL_DIAMETER_MM = 96, DRIVE_GEAR_REDUCTION = 15;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    //dead wheels (thru bore)
    public static final double COUNTS_PER_MOTOR_REV_dead = 8192, WHEEL_DIAMETER_MM_dead = 96;
    public static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    public static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);
    //vuforia
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // vuforia side panels
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

    public VoltageSensor vSensor;
    public boolean lowVoltage = false;
    public double minimumVoltage = 11.5;
    public double currentVoltage;
    public boolean once = false;

    //switchable profile presets
    public String[] driverControls = {"Chase", "Camden","Graden","Kian","Child"},otherControls = driverControls;
    public int baseDriver = 0, baseOther = 1;//list integer of base driver and other controls
    public int dIndex = baseDriver, oIndex = baseOther;//list integer of driver and other controls
    public String currDriver = driverControls[dIndex], currOther = otherControls[oIndex];//list string of driver and other controls
    boolean fieldCentric;
    public boolean optionsHigh1 = false, shareHigh1 = false, optionsHigh2 = false , shareHigh2 = false;

    public void init(HardwareMap ahwMap) {
        updateStatus("Initializing");
        ElapsedTime timer = new ElapsedTime();//declaring the runtime variable
        vSensor = ahwMap.voltageSensor.get("Expansion Hub 2");//getting the voltage sensor\
        getBatteryVoltage();
        //imu
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
        // Declare our motors
        motorFrontLeft = ahwMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = ahwMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = ahwMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = ahwMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor

        //reset all encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontLeft encoder
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackRight encoder
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackLeft encoder
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontRight encoder
        //reversals
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //set all to use encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontLeft encoder to run using encoder
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackLeft encoder to run using encoder
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackRight encoder to run using encoder
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontRight encoder to run using encoder

        //set all to brake when set 0 power
        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        timer.reset();//resetting the runtime variable
        //LED
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
        myOpMode.telemetry.update();
    }
    void getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        double voltage = vSensor.getVoltage();
        if (voltage > 0) {
            result = Math.min(result, voltage);
        }
        lowVoltage = result <= minimumVoltage;
        currentVoltage = result;
    }
    void initTrackables(HardwareMap ahwMap) {//vuforia tags
        webcamName = ahwMap.get(WebcamName.class, "Webcam");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters camParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        camParameters.vuforiaLicenseKey = VUFORIA_KEY;
        camParameters.cameraName = webcamName;
        camParameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(camParameters);
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
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

    public void doBulk() {
        //next two not tested
        bindDriverButtons();
        bindOtherButtons();
        switchProfile();

        switches();//anything that will switch on button press

        if (fieldCentric) {
            gamepadX = myOpMode.gamepad1.left_stick_x;//get the x val of left stick and store
            //myOpMode.telemetry.addData("gamepadX", gamepadX);//tell us what gamepadX is
            gamepadY = -myOpMode.gamepad1.left_stick_y;//get the y val of left stick and store
            //myOpMode.telemetry.addData("gamepadY", gamepadY);//tell us what gamepadY is
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);//get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            //myOpMode.telemetry.addData("gamepadHypot", gamepadHypot);//tell us what gamepadHypot is
            controllerAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX));//Get the angle of the controller stick using arc tangent
            //myOpMode.telemetry.addData("controllerAngle", controllerAngle);//tell us what controllerAngle is
            //might need to change based on corrected heading
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//get and initialize the IMU
            robotDegree = angles.firstAngle;//store robot angle in robotDegree

            //myOpMode.telemetry.addData("robotDegree", robotDegree);//tell us what robotDegree is
            movementDegree = (controllerAngle - robotDegree);//get the movement degree based on the controller vs robot angle
            //myOpMode.telemetry.addData("movementDegree", movementDegree);//tell us what movementDegree is
            xControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;//get the x value of the movement
            //myOpMode.telemetry.addData("xControl", xControl);//tell us what xControl is
            yControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;//get the y value of the movement
            //myOpMode.telemetry.addData("yControl", yControl);//tell us what yControl is
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower;
            backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower;
            frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower;
            backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower;
        } else {
            boolean reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
            yControl = -myOpMode.gamepad1.left_stick_y;
            xControl = myOpMode.gamepad1.left_stick_x;
            if (reverse) {
                yControl = -yControl;
                xControl = -xControl;
            }
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl - xControl + turn) / slowPower;
            backRightPower = (yControl + xControl + turn) / slowPower;
            frontLeftPower = (yControl + xControl - turn) / slowPower;
            backLeftPower = (yControl - xControl - turn) / slowPower;

        }

        // check all the trackable targets to see which one (if any) is visible.
        //targetVisible = false;
        //for (VuforiaTrackable trackable : allTrackables) {
        //    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
        //        myOpMode.telemetry.addData("Visible Target", trackable.getName());
        //        targetVisible = true;
        //        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
        //        if (robotLocationTransform != null) {
        //            lastLocation = robotLocationTransform;
        //        }
        //        break;
        //    }
        //}
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

        power();//sets power to power variables
        once();//runs once

        buildTelemetry();//makes telemetry
    }
    public void once(){
        if (!once) {
            updateStatus("Running");
            findOrientationOffset();
            once = true;
        }

    }
    public void switchProfile(){
        //driver
        if (myOpMode.gamepad1.options && !optionsHigh1){
            if (dIndex == driverControls.length - 1){
                dIndex = 0;
            } else {
                dIndex++;
            }
            currDriver = driverControls[dIndex];
        }
        optionsHigh1 = myOpMode.gamepad1.options;
        if (myOpMode.gamepad1.share && !shareHigh1){
            if (dIndex == 0){
                dIndex = driverControls.length - 1;
            } else {
                dIndex--;
            }
            currDriver = driverControls[dIndex];
        }
        shareHigh1 = myOpMode.gamepad1.share;
        //other
        if (myOpMode.gamepad2.options && !optionsHigh2){
            if (oIndex == otherControls.length - 1){
                oIndex = 0;
            } else {
                oIndex++;
            }
            currOther = otherControls[oIndex];
        }
        optionsHigh2 = myOpMode.gamepad2.options;
        if (myOpMode.gamepad2.share && !shareHigh2){
            if (oIndex == 0){
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
        if (currDriver == driverControls[0]) {
            fieldCentric = false;
        }
        if (currDriver == driverControls[1]) {
            fieldCentric = false;
        }
        if (currDriver == driverControls[2]) {
            fieldCentric = true;
        }
        if (currDriver == driverControls[3]) {
            fieldCentric = true;
        }
    }
    public void bindOtherButtons(){
        //"Chase", "Camden","Graden","Kian","Child"
        if (currOther == otherControls[0]){
        }
        if (currOther == otherControls[1]){
        }
        if (currOther == otherControls[2]){
        }
        if (currOther == otherControls[3]){
        }
    }
    public void antiTip(){
        double maxRoll = 10;
        double minRoll = -10;
        if (roll > maxRoll) {
            //tipped to right
            motorBackLeft.setPower(-1);
            motorFrontLeft.setPower(1);
            if (roll<maxRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        if (roll < minRoll) {
            //tipped to left
            motorBackLeft.setPower(1);
            motorFrontLeft.setPower(-1);
            if (roll>minRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        double maxPitch = 10;
        double minPitch = -10;
        if (pitch > maxPitch) {
            //tipped to front
            motorFrontRight.setPower(1);
            motorFrontLeft.setPower(1);
            telemetry.update();
            if (pitch<maxPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        if (pitch < minPitch) {
            //tipped to back
            motorBackRight.setPower(-1);
            motorBackLeft.setPower(-1);
            if (pitch>minPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
    }

    public void power() {// put all set power here
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void greenRed() {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        sleep(delay * 1000);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        isSolid = true;
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
        } else {
            slowPower = 1;
        }
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

    public void buildTelemetry() {
        myOpMode.telemetry.addData("Status", statusVal);//shows current status
        myOpMode.telemetry.addLine("Drivers")
                .addData("",currDriver)
                .addData("", currOther);
        getBatteryVoltage();
        myOpMode.telemetry.addData("Voltage",currentVoltage);//shows current battery voltage
        myOpMode.telemetry.addData("lowBattery",lowVoltage);
        //myOpMode.telemetry.addData("Timer","%.2f", timer.seconds());//shows current time
        myOpMode.telemetry.addData("Color",LEDcolor);
        myOpMode.telemetry.addData("reversed", reversed);
        myOpMode.telemetry.addData("slowMode", slowModeIsOn);
        getOrientation();
        myOpMode.telemetry.addData("heading", heading);
        myOpMode.telemetry.addData("roll", roll);
        myOpMode.telemetry.addData("pitch", pitch);
        //end testing
        myOpMode.telemetry.addLine("motors: ")
                .addData("front left", motorFrontLeft.getCurrentPosition())
                .addData("front right", motorFrontRight.getCurrentPosition())
                .addData("back left", motorBackLeft.getCurrentPosition())
                .addData("back right", motorBackRight.getCurrentPosition());
        myOpMode.telemetry.addLine("power: ")
                .addData("front left","%2f", frontLeftPower)
                .addData("front right","%2f", frontRightPower)
                .addData("back left","%2f", backLeftPower)
                .addData("back right","%2f", backRightPower);
        teleSpace();
        updateStatus("Running");
        myOpMode.telemetry.update();
    }
    public void getOrientation() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle - primaryHeading;
        roll = angles.secondAngle - primaryRoll;
        pitch = angles.thirdAngle - primaryPitch;
    }
    //meant to fix orientation problems
    public void findOrientationOffset(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        primaryHeading = angles.firstAngle;
        primaryRoll = angles.secondAngle;
        primaryPitch = angles.thirdAngle;
    }

    public String getColor() {
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",
                "RAINBOW_PARTY_PALETTE",
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",
                "BEATS_PER_MINUTE_PARTY_PALETTE",
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
        LEDcolor = favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
        return LEDcolor;
    }

    //random
    public void teleSpace() {
        myOpMode.telemetry.addLine();
    }

    public void updateStatus(String status) {
        statusVal = status;
    }

    //claw
    public double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }
    //encoder

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

            timer.reset();
            motorBackLeft.setPower((speed));
            motorFrontRight.setPower((speed));
            motorFrontLeft.setPower((speed));
            motorBackRight.setPower((speed));
            while (myOpMode.opModeIsActive() &&
                    (timer.seconds() < timeoutS) &&
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

            timer.reset();
            motorBackLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));
            while (myOpMode.opModeIsActive() &&
                    (timer.seconds() < timeoutS) && motorFrontLeft.isBusy()) {

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
    public static void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

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
                    //arm encoder
                }
            } else if (currY > targetY) {
                if (!combo) {
                    encoderDrive(power, -fwdInches, -fwdInches, 6);
                } else {
                    encoderDrive(power, -fwdInches, -fwdInches, 6);
                    //arm encoder
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
        timer.reset();
        while (myOpMode.opModeIsActive()) {// and nothing given back
            if (timer.seconds() > timeoutS) {// is over time
                //auto select
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

        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // need these 2 vars ^
    }

    //
    //public boolean colorInRange(float red, double targetR, float green, double targetG, float blue, double targetB, float range) {
    //    boolean rCheck = false;
    //    boolean gCheck = false;
    //    boolean bCheck = false;
    //    if (targetR - range < red && red < targetR + range) {
    //        rCheck = true;
    //    }
    //    if (targetG - range < green && green < targetG + range) {
    //        gCheck = true;
    //    }
    //    if (targetB - range < blue && blue < targetB + range) {
    //        bCheck = true;
    //    }
    //    return rCheck && gCheck && bCheck;
    //}
    //
    //public void getAllColorR() {
    //    //gives color values
    //    NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
    //    Color.colorToHSV(colorsR.toColor(), hsvValues);
    //    myOpMode.telemetry.addLine()
    //            .addData("Red", "%.3f", colorsR.red)
    //            .addData("Green", "%.3f", colorsR.green)
    //            .addData("Blue", "%.3f", colorsR.blue)
    //            .addData("Hue", "%.3f", hsvValues[0])
    //            .addData("Saturation", "%.3f", hsvValues[1])
    //            .addData("Value", "%.3f", hsvValues[2])
    //            .addData("Alpha", "%.3f", colorsR.alpha);
    //    myOpMode.telemetry.addLine()
    //            .addData("Color", colorName)
    //            .addData("RGB", "(" + redValR + "," + greenValR + "," + blueValR + ")");//shows rgb value
    //}
    //
    //public void getAllColorL() {
    //    //gives color values
    //    NormalizedRGBA colors = colorSensorL.getNormalizedColors();
    //    Color.colorToHSV(colors.toColor(), hsvValues);
    //    myOpMode.telemetry.addLine()
    //            .addData("Red", "%.3f", colors.red)
    //            .addData("Green", "%.3f", colors.green)
    //            .addData("Blue", "%.3f", colors.blue)
    //            .addData("Hue", "%.3f", hsvValues[0])
    //            .addData("Saturation", "%.3f", hsvValues[1])
    //            .addData("Value", "%.3f", hsvValues[2])
    //            .addData("Alpha", "%.3f", colors.alpha);
    //    myOpMode.telemetry.addLine()
    //            .addData("Color", colorName)
    //            .addData("RGB", "(" + redValL + "," + greenValL + "," + blueValL + ")");//shows rgb value
    //}
}
