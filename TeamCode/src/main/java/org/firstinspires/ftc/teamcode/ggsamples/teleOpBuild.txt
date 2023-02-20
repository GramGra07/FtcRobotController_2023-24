//import
package org.firstinspires.ftc.teamcode.ggsamples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;

import java.util.List;
import java.util.Locale;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@TeleOp(name = "teleOpBuild", group = "Pushbot")
@Disabled
public class teleOpBuild extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);
    //initiations
    //camera
    public double spot = 0;
    public String recog = null;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //imu ( inside expansion hub )
    public Orientation angles;     //imu uses these to find angles and classify them
    public Acceleration gravity;    //Imu uses to get acceleration
    public float firstAngle = 0;
    //vars
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180
    //Multiplies this by degrees to see exact position in degrees
    //servo
    public double sparkLongHelp = 0;//will increase to set position higher
    public double sparkShortHelp = 0;
    //public final double spearHelper=180;//full value to go out and in

    //debug
    public final boolean debug_mode = false;//debug mode
    public final boolean testing_mode = true;//test mode
    public String lastButtonPressed = "N/A";//last button pressed//debug will help with this
    double numero = 0;//multiplied for the milli-seconds
    //slowmode
    double slowMode = 0; //0 is off
    final double regular_divider = 1;  //tells slowmode how fast to go when not on
    final double slowMode_divider = 2; //half speed when slowmode
    //endgame
    boolean endgame = false;                 // Use to prevent multiple half-time warning rumbles.
    final double End_Game = 118.0;              // Wait this many seconds before rumble-alert for half-time.
    //telemetry
    public String direction_FW;//string of direction
    public String direction_LR;//string of direction
    public String direction_TLR;//string of direction
    public String slowModeON;//slowmode string ex(on or off)
    public String direction_ANGLE;//string of angle
    public double headingVal = 0;//heading in degrees
    public double alteredHeading = 0;//in case expansion hub is mounted in
    // different direction than facing forward
// init vars (used in initiation process)
    public boolean imuInit = false;
    public String statusVal = "OFFLINE";
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;
    //public double deg=0;
    //gear ratio
    final int gear_ratio = 15;
    public int reduction = 0;
    //custom sleeve vs ftc
    final double sleeve_color_or_sleeve_camera = 2;//1 or 2 respectively
    //
    //private boolean spearIsIn=true;
    //private boolean spearIsOut=false;
    //range
    boolean inRange = false;//tested to see if distance sensor is in range
    boolean updated_inRange = false;//tests again to a boolean for if in range
    boolean updatedHeadingInRange = false;//heading check for low to high

    //run opmode
    @Override
    public void runOpMode() {
        init_controls(true, false);//initiates everything
        robot.init(hardwareMap);
        if (imuInit) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            robot.imu.initialize(parameters);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = robot.imu.getGravity();
        }
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        updateStatus("INIT");//sets status
        //init all motors
        //other initiates
        ElapsedTime runtime = new ElapsedTime();//runtime helps with endgame initiation and cues
        if (imuInit) {//will set up everything for imu
            robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            composeTelemetry();
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            runtime.reset();
            getRuntime();//gets runtime
            init_controls(false, false);//only imu if first init//initiates everything
            double y = gamepad1.left_stick_y; // Remember, this is reversed!//forward backward
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing//left right
            double rx = -gamepad1.right_stick_x;//turning
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);//gets max value
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            //slowmode
            if (gamepad1.b && slowMode == 0) {//checks for button press and slowmode off
                slowMode = 1;//sets slowmode to on
                if (debug_mode) {
                    getLastButtonPress("b");
                }
            } else if (gamepad1.b && slowMode == 1) {//checks for button press and slowmode on
                slowMode = 0;//sets slowmode to off
                if (debug_mode) {
                    getLastButtonPress("b");
                }
            }
            if (slowMode == 1) {
                backRightPower /= slowMode_divider;//divides power by the divider
                backLeftPower /= slowMode_divider;//divides power by the divider
                frontRightPower /= slowMode_divider;//divides power by the divider
                frontLeftPower /= slowMode_divider;//divides power by the divider
            } else {
                backRightPower /= regular_divider;
                backLeftPower /= regular_divider;
                frontRightPower /= regular_divider;
                frontLeftPower /= regular_divider;
            }
            //endgame init
            if ((runtime.seconds() > End_Game) && !endgame) {//sets endgame
                endgame = true;
            }
            if (!endgame) {
                telemetry.addData(">", "Almost ENDGAME: %3.0f Sec \n", (End_Game - runtime.seconds()));//shows time til endgame
            }
            //
            //sets power to respective motors
            //setAllDrivePower(frontRightPower,frontLeftPower,backRightPower,backLeftPower);
            robot.sparkLong.setPower(sparkLongHelp);
            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorBackRight.setPower(backRightPower);
            teleSpace();//puts a space in telemetry
            runVu();
            telemetry.update();
            sleep(50);
            //if (spearIsIn) {
            //    spearIsOut = false;
            //}
            //else if(spearIsOut) {
            //    spearIsIn = false;
            //}
        }
    }

    //not even close to finished
    //sense sleeve
//    public void sleeveSense(){
//        showFeedback();
//        robot.spear.setPosition(setServo((int) spearHelper));
//        while (CM_distance1>=5) {
//            //move closer
//        }while (CM_distance1<=5){
//           //move back
//        }
//        if( CM_distance1==1){//prime distance
//            if (colorName.equals( "red")){
//                //do this
//            }
//            if (colorName.equals( "blue")){
//                //do this
//            }
//            if (colorName.equals( "black")){
//                //do this
//            }
//        }
//        robot.spear.setPosition(setServo((int) -spearHelper));
//    }
//experimental
    public void gearRatio(int ratio) {
        reduction = 20 / ratio;
    }

    public void setAllDrivePower(double fr, double fl, double br, double bl) {
        fr /= reduction;
        fl /= reduction;
        br /= reduction;
        bl /= reduction;
        robot.motorFrontRight.setPower(fr);
        robot.motorFrontLeft.setPower(fl);
        robot.motorBackRight.setPower(br);
        robot.motorBackLeft.setPower(bl);
    }

    public void updateStatus(String status) {
        statusVal = status;
    }//set a new controller/game status

    //setServo//this sets the servo to a position based off a given degree
    //ex: servo.setPosition(setServo(90))
    public double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public double milli_seconds(int seconds) {
        numero = 1000 * seconds;
        return numero;
    }

    public void getLastButtonPress(String button) {
        lastButtonPressed = button;
    }//gets last button press

    public void dance(String direction_1) {//-1=back//1=forward//fun little thing we learned from others
        updateStatus("Dancing");
        if (direction_1.equals("backwards")) {
            robot.motorFrontLeft.setPower(1);
            robot.motorBackLeft.setPower(-1);
            robot.motorFrontRight.setPower(1);
            robot.motorBackRight.setPower(-1);
        }
        if (direction_1.equals("forwards")) {
            robot.motorFrontLeft.setPower(-1);
            robot.motorBackLeft.setPower(1);
            robot.motorFrontRight.setPower(-1);
            robot.motorBackRight.setPower(1);
        }
    }

    //non-expiremental
//init
    //will initiate based on variables and assign variables
    public void init_controls(boolean first, boolean reduction) {
        if (reduction) {
            gearRatio(gear_ratio);
        }
        if (first) {
            updateStatus("INIT");
            //resetEncoder();
        } else {
            updateStatus("RUNNING");
        }
        showFeedback();//gives feedback on telemetry
    }

    //telemetry
    //make space in telemetry read-out
    public void teleSpace() {
        telemetry.addLine();
    }

    //telemetry additions
    public void showFeedback() {
        //get variables for telemetry
        if (debug_mode) {
            telemetry.addData(" |__|                  ", "");
            telemetry.addData(" (o-)                   ", "");
            telemetry.addData("//||\\                  ", "");
        }
        telemetry.addData("Status", statusVal);//shows current status
        //gets direction vertical
        if (testing_mode) {
            telemetry.addData(" Testing Mode 1...2...3...                  ", "");
            if (gamepad1.left_stick_y < 0) {
                direction_FW = "forward";
            }
            if (gamepad1.left_stick_y > 0) {
                direction_FW = "backward";
            }
            if (gamepad1.left_stick_y == 0) {
                direction_FW = "idle";
            }

            //gets direction horizontal
            if (gamepad1.left_stick_x > 0) {
                direction_LR = "right";
            }
            if (gamepad1.left_stick_x < 0) {
                direction_LR = "left";
            }
            if (gamepad1.left_stick_x == 0) {
                direction_LR = "idle";
            }
            //gets turn angle
            if (gamepad1.right_stick_x > 0) {
                direction_TLR = "right";
            }
            if (gamepad1.right_stick_x < 0) {
                direction_TLR = "left";
            }
            if (gamepad1.right_stick_x == 0) {
                direction_TLR = "idle";
            }
            //shows slowmode status
            if (slowMode == 1) {
                slowModeON = "True";
            } else {
                slowModeON = "False";
            }
        }
        //direction heading
        if (imuInit) {
            getHeading();
            telemetry.addData("Heading", "%.1f", headingVal);
            telemetry.addData("Heading Direction", direction_ANGLE);
            if (testing_mode) {
                if (headingVal > 45 && headingVal < 135) {
                    direction_ANGLE = "right";
                }
                if (headingVal > -45 && headingVal < 45) {
                    direction_ANGLE = "forward";
                }
                if (headingVal < 135 && headingVal > -135) {
                    direction_ANGLE = "backwards";
                }
                if (headingVal < -45 && headingVal > -135) {
                    direction_ANGLE = "left";
                }
            }
        }

        //shows all previously defined values
        if (testing_mode) {
            telemetry.addLine()
                    .addData("direction", direction_FW)
                    .addData("strafe", direction_LR)
                    .addData("turn", direction_TLR)
                    .addData("r trigger", "%.2f", gamepad1.right_trigger)
                    .addData("l trigger", "%.2f", gamepad1.left_trigger);
            teleSpace();
            telemetry.addData("slowMode", slowModeON);
        }
        teleSpace();

    }

    //imu telemetry
    void composeTelemetry() {
        getHeading();
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = robot.imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
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

    //imu
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void getHeading() {
        firstAngle = angles.firstAngle;
        headingVal = firstAngle + alteredHeading;
    }
    //range
    //gets the values and finds if it is in a range of max to min

    // if (checkHeading(90,0))==True{}
    public boolean checkHeading(int maxH, int minH) {
        getHeading();
        resetRanges();
        updatedHeadingInRange = headingVal >= minH && headingVal <= maxH;
        return updatedHeadingInRange;
    }

    //resets range
    public void resetRanges() {
        updated_inRange = false;
        inRange = false;
    }

    //camera
    public void runVu() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    double centerX = (recognition.getLeft() + recognition.getRight()) / 2;
                    double centerY = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", centerX, centerY);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    recog = (recognition.getLabel());
                    if (spot == 0) {
                        if (recog == "1 Bolt") {
                            spot += 1;
                            //red
                        }
                        if (recog == "2 Bulb") {
                            spot += 2;
                            //blue
                        }
                        if (recog == "3 Panel") {
                            spot += 3;
                            //black
                        }
                        telemetry.addData("Spot:", String.valueOf(spot), (recog));
                    }

                }
            }
        }
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}