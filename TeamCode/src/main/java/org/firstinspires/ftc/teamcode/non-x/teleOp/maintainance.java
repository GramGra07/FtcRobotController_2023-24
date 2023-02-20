package org.firstinspires.ftc.teamcode.teleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "maintainanceMode", group = "Robot")
@Disabled
public class maintainance extends robotCentric {
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

    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180

    public DigitalChannel red1;
    public DigitalChannel green1;
    public DigitalChannel red2;
    public DigitalChannel green2;
    public DigitalChannel red3;
    public DigitalChannel green3;
    public DigitalChannel red4;
    public DigitalChannel green4;
    NormalizedColorSensor colorSensor;//declaring the colorSensor variable
    public TouchSensor touchSensor;
    public TouchSensor touchSensorL;
    public TouchSensor touchSensorClaw;
    public TouchSensor touchSensorEject;
    public boolean armUp = false;
    public boolean clawOpen = false;
    public boolean tapeOut = false;
    public final int timeout = 1;
    public final int delay = 1;
    RevBlinkinLedDriver lights;
    public boolean isSolid = false;
    public String color = "none";

    @Override
    public void runOpMode() {
        red1 = hardwareMap.get(DigitalChannel.class, "red1");//getting the red1 light
        green1 = hardwareMap.get(DigitalChannel.class, "green1");//getting the green1 light
        red2 = hardwareMap.get(DigitalChannel.class, "red2");//getting the red2 light
        green2 = hardwareMap.get(DigitalChannel.class, "green2");//getting the green2 light
        red3 = hardwareMap.get(DigitalChannel.class, "red3");//getting the red3 light
        green3 = hardwareMap.get(DigitalChannel.class, "green3");//getting the green3 light
        red4 = hardwareMap.get(DigitalChannel.class, "red4");//getting the red4 light
        green4 = hardwareMap.get(DigitalChannel.class, "green4");//getting the green4 light
        touchSensor = hardwareMap.get(TouchSensor.class, ("touchSensor"));
        touchSensorL = hardwareMap.get(TouchSensor.class, ("touchSensorL"));
        touchSensorClaw = hardwareMap.get(TouchSensor.class, ("touchSensorClaw"));
        touchSensorEject = hardwareMap.get(TouchSensor.class, ("touchSensorEject"));
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        tapeMeasure = hardwareMap.get(DcMotor.class, "tapeMeasure");
        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tapeMeasure.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tapeMeasure.setDirection(DcMotor.Direction.REVERSE);
        sparkLong.setZeroPowerBehavior(BRAKE);
        tapeMeasure.setZeroPowerBehavior(BRAKE);
        red1.setMode(DigitalChannel.Mode.OUTPUT);//setting the red1 light to output
        green1.setMode(DigitalChannel.Mode.OUTPUT);//setting the green1 light to output
        red2.setMode(DigitalChannel.Mode.OUTPUT);//setting the red2 light to output
        green2.setMode(DigitalChannel.Mode.OUTPUT);//setting the green2 light to output
        red3.setMode(DigitalChannel.Mode.OUTPUT);//setting the red3 light to output
        green3.setMode(DigitalChannel.Mode.OUTPUT);//setting the green3 light to output
        red4.setMode(DigitalChannel.Mode.OUTPUT);//setting the red4 light to output
        green4.setMode(DigitalChannel.Mode.OUTPUT);//setting the green4 light to output
        ElapsedTime runtime = new ElapsedTime();
        if (isStopRequested()) return;
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (isSolid) {
                sleep(delay * 1000);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
                isSolid = false;
            }
            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
            if (runtime.seconds() > timeout) { // if runtime is greater than timeout, allow it to switch
                //should prevent it from just cycling on and off
                if (touchSensor.isPressed()) {
                    armUp = !armUp;
                    greenRed();
                    runtime.reset();
                }
                if (touchSensorClaw.isPressed()) {
                    clawOpen = !clawOpen;
                    greenRed();
                    runtime.reset();
                }
                if (touchSensorEject.isPressed()) {
                    greenRed();
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
                    runtime.reset();
                }
                if (touchSensorL.isPressed()) {
                    greenRed();
                    tapeOut = !tapeOut;
                    runtime.reset();
                }
            }
            if (tapeOut) {
                tapeEncoder((int) (countsPerInchTape * 18), 1, 6, false);//go out
                green3.setState(true);
                red3.setState(false);
            } else {
                tapeEncoder(0, 1, 6, true);// come in
                green3.setState(false);
                red3.setState(true);
            }
            if (armUp) {
                armEncoder(1800, 1, 6, false);
                green1.setState(true);
                red1.setState(false);
            } else {
                armEncoder(0, 0.8, 6, true);
                green1.setState(false);
                red1.setState(true);
            }
            if (clawOpen) {
                openClaw();
                green2.setState(false);
                red2.setState(true);
            } else {
                closeClaw();
                green2.setState(true);
                red2.setState(false);
            }
            telemetry.addData("armUp", armUp);
            telemetry.addData("clawOpen", clawOpen);
            telemetry.addData("color", color);
            telemetry.update();
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
        String c = favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
        color = c;
        return c;
    }
}
