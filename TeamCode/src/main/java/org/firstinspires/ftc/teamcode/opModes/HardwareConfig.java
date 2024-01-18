//import
package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.Drivers.bindDriverButtons;
import static org.firstinspires.ftc.teamcode.Drivers.currDriver;
import static org.firstinspires.ftc.teamcode.Drivers.currOther;
import static org.firstinspires.ftc.teamcode.Drivers.fieldCentric;
import static org.firstinspires.ftc.teamcode.Drivers.switchProfile;
import static org.firstinspires.ftc.teamcode.EOCVWebcam.cam2_N;
import static org.firstinspires.ftc.teamcode.Operator.bindOtherButtons;
import static org.firstinspires.ftc.teamcode.Sensors.currentVoltage;
import static org.firstinspires.ftc.teamcode.Sensors.getBatteryVoltage;
import static org.firstinspires.ftc.teamcode.Sensors.loadDistance;
import static org.firstinspires.ftc.teamcode.Sensors.lowVoltage;
import static org.firstinspires.ftc.teamcode.Sensors.operateClawByDist;
import static org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.setUpFile;
import static org.firstinspires.ftc.teamcode.UtilClass.FileWriterFTC.writeToFile;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.setDirectionR;
import static org.firstinspires.ftc.teamcode.UtilClass.MotorUtil.zeroPowerBrake;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.servoFlipVal;
import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.setServo;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime.regularLoopTime;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.LoopTime.useLoopTime;
import static org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig.useAutoClose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.UtilClass.Blink;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.IsBusy;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.PastPotent;
import org.firstinspires.ftc.teamcode.UtilClass.varStorage.varConfig;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.DistanceStorage;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.advanced.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.util.List;

public class HardwareConfig {//this is an external opMode that can have public variables used by everything
    public static boolean useFileWriter = varConfig.useFileWriter;
    public static boolean multipleDrivers = varConfig.multipleDrivers;
    public static String statusVal = "OFFLINE";
    public static Servo claw1 = null, claw2 = null, flipServo = null, airplaneServo = null;
    public static DcMotor motorFrontLeft = null, motorBackLeft = null, motorFrontRight = null, motorBackRight = null, motorLift = null, motorExtension = null, motorRotation = null;
    public static RevBlinkinLedDriver lights;
    public int slowMult = varConfig.slowMult, slowPower;
    public static boolean slowModeIsOn = false, reversed;
    public double xControl, yControl, frontRightPower, frontLeftPower, backRightPower, backLeftPower;
    public static double liftPower = 0, extensionPower = 0, rotationPower = 0;
    public static double loops = 0;
    public static double distance1 = 0, distance2 = 0;
    public static boolean claw1Possessed = false, claw2Possessed = false;
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
    public static AnalogInput potentiometer;
    //    public DigitalChannel limitSwitch;
    public static VoltageSensor vSensor;
    public static MecanumDrive drive = null;
    public static double thisDist = 0;
    public static final ElapsedTime timer = new ElapsedTime();
    static FileWriter fileWriter;

    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public HardwareConfig(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public boolean once = false;
    public static PIDController pidExtension = new PIDController(
            0.1, // Proportional gain
            0.1, // Integral gain
            0.1 // Derivative gain
    );
    public static VisionPortal visionPortal; // vision portal for the webcam
    public static AprilTagProcessor aprilTagProcessor; // april tag processor for the vision portal
    public static DistanceSensor distanceSensor1;
    public static DistanceSensor distanceSensor2;

    public static final String currentVersion = "5.0.0";

    //init
    public static void init(HardwareMap ahwMap, boolean auto) {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        thisDist = 0;
        setUpFile(fileWriter);
        updateStatus("Initializing");
        drive = new MecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        ElapsedTime timer = new ElapsedTime();//declaring the runtime variable
        vSensor = ahwMap.voltageSensor.get("Expansion Hub 2");//getting the voltage sensor
        getBatteryVoltage(vSensor);
        lights = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        // rev potentiometer //analog
        potentiometer = ahwMap.get(AnalogInput.class, "potent");
        //magnetic limit switch //digital is pressed
//        limitSwitch = ahwMap.get(DigitalChannel.class, "limitSwitch");
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
        motorExtension = ahwMap.get(DcMotor.class, "slideMotor");
        motorRotation = ahwMap.get(DcMotor.class, "flipperMotor");
        claw1 = ahwMap.get(Servo.class, "claw1");
        claw2 = ahwMap.get(Servo.class, "claw2");
        flipServo = ahwMap.get(Servo.class, "flipServo");
        airplaneServo = ahwMap.get(Servo.class, "airplaneServo");
        distanceSensor1 = ahwMap.get(DistanceSensor.class, "C1");
        distanceSensor2 = ahwMap.get(DistanceSensor.class, "C2");
        //encoders
        //reversals
        setDirectionR(motorBackLeft);
        setDirectionR(motorRotation);
        setDirectionR(motorLift);
        //set all to brake when set 0 power
        zeroPowerBrake(motorBackRight);
        zeroPowerBrake(motorBackLeft);
        zeroPowerBrake(motorFrontLeft);
        zeroPowerBrake(motorFrontRight);
        zeroPowerBrake(motorExtension);
        zeroPowerBrake(motorRotation);
        zeroPowerBrake(motorLift);
        closeClaw(claw1);
        closeClaw(claw2);
        PastPotent.pastPotentVal = Sensors.getPotentVal(potentiometer);

        motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        cRE = new Gamepad.RumbleEffect.Builder()
//                .addStep(1.0, 1.0, 250)
//                .build();
        if (!auto) {
            aprilTagProcessor = new AprilTagProcessor.Builder() // april tag processor initialization
                    .setDrawAxes(true) // draw axes on the april tag
                    .setDrawCubeProjection(false) // don't draw cube projection on the april tag
                    .setDrawTagOutline(true) // draw tag outline on the april tag
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // set the tag family to 36h11
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // set the tag library to the center stage tag library
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // set the output units to inches and degrees
                    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.
                    .build();
            VisionPortal.Builder builder = new VisionPortal.Builder(); // vision portal builder initialization
            builder.setCamera(ahwMap.get(WebcamName.class, cam2_N)); // set the camera to webcam 1
            builder.addProcessor(aprilTagProcessor); // add the april tag processor to the vision portal
            visionPortal = builder.build(); // build the vision portal
            visionPortal.setProcessorEnabled(aprilTagProcessor, false); // disable the april tag processor
        }

        timer.reset();
        Sensors.ledIND(green1, red1, true);
        Sensors.ledIND(green2, red2, true);
        Sensors.ledIND(green3, red3, true);
        Sensors.ledIND(green4, red4, true);
        Blink.setLights("HOT_PINK", true);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color", LEDcolor);
        telemetry.addData("Version", currentVersion);
        telemetry.addData("Voltage", "%.2f", currentVoltage);
        if (lowVoltage) {
            telemetry.addData("lowBattery", "true");
        }
        if (!auto) {
            telemetry.update();
        }
    }

    //code to run all drive functions
    public void doBulk() {
        once(myOpMode);//runs once
        periodically();//runs every loop
        bindDriverButtons(myOpMode, drive);
        bindOtherButtons(myOpMode, drive);
        if (multipleDrivers) {
            switchProfile(myOpMode);
        }
        drive(fieldCentric);
//        updatePoseByAprilTag(drive);
        power();//sets power to power variables
        buildTelemetry();//makes telemetry
        loops++;
    }

    public void once(OpMode myOpMode) {
        if (!once) {
            Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
            // Telemetry telemetry = myOpMode.telemetry;
            telemetry.clearAll();

            updateStatus("Running");
//            int duration = 180000000;
//            myOpMode.gamepad1.setLedColor(229, 74, 161, duration);
//            myOpMode.gamepad2.setLedColor(54, 69, 79, duration);
            once = true;
        }
    }

    public void periodically() {
        if (useLoopTime) {
            if (loops % regularLoopTime == 0) { // happens every regularLoopTime, loops
                loadDistance();
                operateClawByDist();
            }
        } else {
            if (useAutoClose) {
                loadDistance();
                operateClawByDist();
            }
        }
    }

    public void drive(boolean fieldCentric) {
        if (fieldCentric) {
            gamepadX = myOpMode.gamepad1.left_stick_x;//get the x val of left stick and store
            gamepadY = -myOpMode.gamepad1.left_stick_y;//get the y val of left stick and store
            gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);//get the
            // hypotenuse of the x and y values,clip it to a max of 1 and store
            controllerAngle = Math.toDegrees(Math.atan2(gamepadY, gamepadX));//Get the angle of the controller stick using arc tangent
            robotDegree = Math.toDegrees(drive.getPoseEstimate().getHeading()); // change to imu
            movementDegree = (controllerAngle - robotDegree);//get the movement degree based on the controller vs robot angle
            xControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;//get the x value of the movement
            yControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;//get the y value of the movement
            double turn = -myOpMode.gamepad1.right_stick_x;
            frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn) / slowPower;
            backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn) / slowPower;
            frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn) / slowPower;
            backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn) / slowPower;
        } else {
//            reverse = myOpMode.gamepad1.touchpad_finger_1_x > 0.5;//0,1 left to right
//            reversed = reverse;
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
        if (!IsBusy.isAutoInTeleop) {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
        motorLift.setPower(liftPower);
        motorExtension.setPower(extensionPower);
        motorRotation.setPower(rotationPower);
        flipServo.setPosition(setServo(servoFlipVal));
    }

    public void buildTelemetry() {
        Telemetry telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        //////////

//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//        teleSpace();
//        teleSpace();

        //////////
        //Telemetry telemetry = myOpMode.telemetry;
        telemetry.addData("Drivers", currDriver + " " + currOther);
        getBatteryVoltage(vSensor);
        telemetry.addData("Voltage", "%.1f", currentVoltage);//shows current battery voltage
        if (lowVoltage) {
            telemetry.addData("", "We have a low battery");
        }
        telemetry.addData("potentiometer", "%.1f", Sensors.getPotentVal(potentiometer));
        telemetry.addData("Distance 1", distance1);
        telemetry.addData("Distance 2", distance2);

        telemetry.addData("Color", LEDcolor);
        if (reversed) {
            telemetry.addData("reversed", "");
        }
        if (slowModeIsOn) {
            telemetry.addData("slowMode", "");
        }
        if (useAutoClose) {
            telemetry.addData("autoClose", "");
        }
        telemetry.addData("Extension", motorExtension.getCurrentPosition());
        teleSpace();
        telemetry.addData("x", "%.1f", drive.getPoseEstimate().getX());
        telemetry.addData("y", "%.1f", drive.getPoseEstimate().getY());
        telemetry.addData("heading", "%.1f", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        teleSpace();
        telemetry.addData("thisDistance (in)", "%.1f", thisDist);
        telemetry.addData("totalDistance (in)", "%.1f", DistanceStorage.totalDist);
        teleSpace();
        telemetry.addData("Timer", "%.1f", timer.seconds());//shows current time
        telemetry.addData("Loops", "%.1f", loops);
        telemetry.addData("FPS", "%.1f", loops / timer.seconds());
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

    public void updatePoseByAprilTag(MecanumDrive drive) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections.size() > 0) {
            if (currentDetections.size() == 1) {
                for (AprilTagDetection detection : currentDetections) {
                    drive.setPoseEstimate(new Pose2d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
                    PoseStorage.currentPose = drive.getPoseEstimate();
                }
            } else {
                Pose2d pose = new Pose2d();
                for (AprilTagDetection detection : currentDetections) {
                    if (pose.equals(new Pose2d())) {
                        pose = new Pose2d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
                    } else {
                        pose = new Pose2d((pose.getX() + detection.ftcPose.x) / 2, (pose.getY() + detection.ftcPose.y) / 2, (pose.getHeading() + detection.ftcPose.yaw) / 2);
                    }
                }
                drive.setPoseEstimate(pose);
                PoseStorage.currentPose = drive.getPoseEstimate();
            }
        }
    }

    public static void updateStatus(String status) {
        statusVal = status;
    }

}
